//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod qe;

use defmt::*;
use defmt_rtt as _;
use embedded_hal::PwmPin;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::timer::CountDown;
use fugit::ExtU64;
use panic_probe as _;
use pimoroni_servo2040::hal::clocks::SystemClock;
use pimoroni_servo2040::hal::dma::{Channel, ChannelIndex, DMAExt, CH0, CH1};
use pimoroni_servo2040::hal::gpio::{Error as GpioError, FunctionConfig, FunctionPio0};
use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO, SM0};
use pimoroni_servo2040::hal::{self, pac, Clock};
use pimoroni_servo2040::pac::{interrupt, PIO0};
use servo_pio::calibration::{AngularCalibration, Calibration};
use servo_pio::pwm_cluster::{dma_interrupt, GlobalState, GlobalStates, Handler};
use servo_pio::servo_cluster::{
    ServoCluster, ServoClusterBuilder, ServoClusterBuilderError, ServoData,
};

const NUM_SERVOS: usize = 1;
const NUM_CHANNELS: usize = 12;
static mut STATE1: Option<GlobalState<CH0, CH1, PIO0, SM0>> = {
    const NONE_HACK: Option<GlobalState<CH0, CH1, PIO0, SM0>> = None;
    NONE_HACK
};
static mut GLOBALS: GlobalStates<NUM_CHANNELS> = {
    const NONE_HACK: Option<&'static mut dyn Handler> = None;
    GlobalStates {
        states: [NONE_HACK; NUM_CHANNELS],
    }
};

#[pimoroni_servo2040::entry]
fn main() -> ! {
    info!("entering main");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let sio = hal::Sio::new(pac.SIO);

    let clocks = hal::clocks::init_clocks_and_plls(
        pimoroni_servo2040::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = pimoroni_servo2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let servo_pins: [_; NUM_SERVOS] = [
        ServoData {
            pin: pins.servo4.into_mode::<FunctionPio0>().into(),
            calibration: Calibration::builder(AngularCalibration::default())
                .limit_lower()
                .limit_upper()
                .build(),
        },
    ];
    info!("pins");

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    // Use a different pio for the leds because they run at a different
    // clock speed.
    let (mut pio1, sm10, _, _, _) = pac.PIO1.split(&mut pac.RESETS);

    let dma = pac.DMA.split(&mut pac.RESETS);

    // Configure the Timer peripheral in count-down mode.
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();
    info!("timer");

    let mut servo_cluster = match build_servo_cluster(
        &mut pio0,
        sm0,
        (dma.ch0, dma.ch1),
        servo_pins,
        #[cfg(feature = "debug_pio")]
        pins.scl.into_mode::<FunctionPio0>().into(),
        clocks.system_clock,
        unsafe { &mut STATE1 },
    ) {
        Ok(cluster) => cluster,
        Err(_e) => {
            defmt::error!("Failed to build servo cluster");
            #[allow(clippy::empty_loop)]
            loop {}
        }
    };
    info!("cluster");

    // Init PWMs ------------------------------------------
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.set_div_int(1);
    const MAX_PWM: u16 = 4096; // 15kHz PWM (must be in the range of 5-100kHz)
    pwm.set_top(MAX_PWM);
    pwm.enable();

    // Output channel B on PWM4 to the LED pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.adc_addr_2);
    info!("pwm");

    // Init encoder ---------------------------------------
    // GP9/GP10 for A/B pins + GP12 for pushbutton contact
    pins.servo10.into_pull_up_input();
    pins.servo11.into_pull_up_input();
    let (qe_sm, mut qe_rx, mut qe_tx)
    = qe::quadrature_encoder(&mut pio1, sm10, 9);
    qe_sm.start();

    // Init switch ---------------------------------------
    let switch = pins.servo13.into_pull_up_input();

    // Unmask the DMA interrupt so the handler can start running. This can only
    // be done after the servo cluster has been built.
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    const MIN_PULSE: f32 = 1500.0;
    const MID_PULSE: f32 = 2000.0;
    const MAX_PULSE: f32 = 2500.0;
    const QE_MIN: i32 = -100;
    const QE_MAX: i32 = 100;

    let movement_delay = 20.millis();

    // We need to use the indices provided by the cluster because the servo pin
    // numbers do not line up with the indices in the clusters and PIO.
    let [servo1] = servo_cluster.servos();

    servo_cluster.set_pulse(servo1, MAX_PULSE, false);
    info!("set_pulse");
    count_down.start(movement_delay * 5);

    let mut brightness = 0;
    let mut going_up = true;
    const STEP_SIZE: u16 = 128;
    let mut qe_val = 0;
    let mut switch_pressed = false;
    info!("entering loop");
    #[allow(clippy::empty_loop)]
    loop {
        // handle QE
        qe_tx.write(1);

        // handle PWM
        channel.set_duty(brightness);
        if going_up {
            if brightness < MAX_PWM {
                brightness = brightness.saturating_add(STEP_SIZE);
            } else {
                going_up = false;
            }
        } else {
            if brightness > 0 {
                brightness = brightness.saturating_sub(STEP_SIZE);
            } else {
                going_up = true;
            }
        }
        count_down.start(movement_delay);
        let _ = nb::block!(count_down.wait());

        if !switch_pressed && switch.is_low().unwrap_or(false) {
            switch_pressed = true;
            info!("press!");
        } else if switch.is_high().unwrap_or(false) {
            switch_pressed = false;
        }

        while qe_rx.is_empty() {
            // wait
        }
        match qe_rx.read() {
            Some(val) => {
                qe_val = i32::from_le_bytes(val.to_le_bytes());
            },
            _ => info!("QE error"),
        }
        // handle servo
        let mut pulse: f32 = (qe_val as f32) * (MAX_PULSE - MIN_PULSE) / (QE_MAX as f32 - QE_MIN as f32) + MID_PULSE;
        pulse = pulse.clamp(MIN_PULSE, MAX_PULSE);
        servo_cluster.set_pulse(servo1, pulse, false);
        servo_cluster.load();
    }
}

// #[derive(Format)]
enum BuildError {
    Gpio(GpioError),
    Build(ServoClusterBuilderError),
}

fn build_servo_cluster<C1, C2, P, SM>(
    pio: &mut PIO<P>,
    sm: UninitStateMachine<(P, SM)>,
    dma_channels: (Channel<C1>, Channel<C2>),
    servo_data: [ServoData<AngularCalibration>; NUM_SERVOS],
    #[cfg(feature = "debug_pio")] side_set_pin: DynPin,
    system_clock: SystemClock,
    state: &'static mut Option<GlobalState<C1, C2, P, SM>>,
) -> Result<ServoCluster<NUM_SERVOS, P, SM, AngularCalibration>, BuildError>
where
    C1: ChannelIndex,
    C2: ChannelIndex,
    P: PIOExt + FunctionConfig,
    SM: StateMachineIndex,
{
    #[allow(unused_mut)]
    let mut builder: ServoClusterBuilder<
        '_,
        AngularCalibration,
        C1,
        C2,
        P,
        SM,
        NUM_SERVOS,
        NUM_CHANNELS,
    > = ServoCluster::<NUM_SERVOS, P, SM, AngularCalibration>::builder(
        pio,
        sm,
        dma_channels,
        unsafe { &mut GLOBALS },
    )
    .pins_and_calibration(servo_data)
    .map_err(BuildError::Gpio)?;
    #[cfg(feature = "debug_pio")]
    {
        builder = builder
            .side_set_pin(side_set_pin)
            .map_err(BuildError::Gpio)?;
    }
    builder
        .pwm_frequency(50.0)
        .build(&system_clock, state)
        .map_err(BuildError::Build)
}

#[interrupt]
fn DMA_IRQ_0() {
    critical_section::with(|_| {
        // Safety: we're within a critical section, so nothing else will modify global_state.
        dma_interrupt(unsafe { &mut GLOBALS });
    });
}