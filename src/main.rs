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
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::timer::CountDown;
use fugit::ExtU64;
use panic_probe as _;
use pimoroni_servo2040::hal::clocks::SystemClock;
use pimoroni_servo2040::hal::dma::{Channel, ChannelIndex, DMAExt, CH0, CH1};
use pimoroni_servo2040::hal::gpio::{Error as GpioError, FunctionConfig, FunctionPio0};
use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO, SM0};
use pimoroni_servo2040::hal::{self, pac};
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

#[derive(Debug, Format)]
enum OpMode {
    Angle,
    Brightness,
    Pixel,
}

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
    let pwm = &mut pwm_slices.pwm3;
    pwm.set_ph_correct();
    pwm.set_div_int(1);
    const MAX_PWM: u16 = 4096; // 15kHz PWM (must be in the range of 5-100kHz)
    pwm.set_top(MAX_PWM);
    pwm.enable();

    // Output channel B on PWM4 to the LED pin
    let channel = &mut pwm.channel_a;
    let mut pwm_pin = Some(channel.output_to(pins.adc_addr_0));
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

    // Lateral light angle -------------------------------
    let mut clk = pins.scl.into_push_pull_output_in_state(hal::gpio::PinState::Low);
    // let mut data = pins.adc_addr_0.into_push_pull_output_in_state(hal::gpio::PinState::Low);
    let mut data_pin: Option<rp2040_hal::gpio::Pin<_, _>> = None;

    // Unmask the DMA interrupt so the handler can start running. This can only
    // be done after the servo cluster has been built.
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    const MIN_PULSE: f32 = 1500.0;
    const MID_PULSE: f32 = 2000.0;
    const MAX_PULSE: f32 = 2500.0;
    const ANGLE_MIN: i32 = -100;
    const ANGLE_MAX: i32 = 100;
    const ANGLE_INCREMENT: i32 = 1;
    const BRIGHTNESS_INCREMENT: u16 = MAX_PWM / 128;

    let movement_delay = 20.millis();
    let shift_delay = 100.micros();

    // We need to use the indices provided by the cluster because the servo pin
    // numbers do not line up with the indices in the clusters and PIO.
    let [servo1] = servo_cluster.servos();

    servo_cluster.set_pulse(servo1, MAX_PULSE, false);
    info!("set_pulse");
    count_down.start(movement_delay * 5);

    let mut brightness = MAX_PWM;
    let mut switch_pressed = false;
    let mut mode = OpMode::Angle;
    let mut last_delta = 0;
    let mut angle = 0;
    let mut pixel_state = 1u16;
    let mut pix_ud_count = 0i32;
    const RELAXATION_INTERVAL: usize = 30;
    const UD_THRESHOLD: usize = 3;
    let mut r_interval = 0;

    channel.set_duty(brightness);
    servo_cluster.set_pulse(servo1, MID_PULSE, false);
    servo_cluster.load();

    info!("entering loop");
    #[allow(clippy::empty_loop)]
    loop {
        // handle QE
        qe_tx.write(1);

        // handle PWM
        count_down.start(movement_delay);
        let _ = nb::block!(count_down.wait());

        if !switch_pressed && switch.is_low().unwrap_or(false) {
            switch_pressed = true;
            // cycle through the modes
            mode = match mode {
                OpMode::Angle => OpMode::Brightness,
                OpMode::Brightness => {
                    let convert_to_pin = pwm_pin.take().unwrap();
                    data_pin = Some(convert_to_pin.into_push_pull_output());
                    OpMode::Pixel
                },
                OpMode::Pixel => {
                    if let Some(data) = data_pin.as_mut() {
                        data.set_low().unwrap();
                        count_down.start(shift_delay);
                        let _ = nb::block!(count_down.wait());
                        // all on again
                        clk.set_low().unwrap();
                        for _ in 0..5 {
                            data.set_low().unwrap();
                            count_down.start(shift_delay);
                            let _ = nb::block!(count_down.wait());
                            clk.set_high().unwrap();
                            count_down.start(shift_delay);
                            let _ = nb::block!(count_down.wait());
                            clk.set_low().unwrap();
                        }
                    }
                    let convert_to_pin = data_pin.take().unwrap();
                    pwm_pin = Some(channel.output_to(convert_to_pin));
                    OpMode::Angle
                },
            };
            info!("switching to {:?}", mode);
        } else if switch.is_high().unwrap_or(false) {
            switch_pressed = false;
        }

        while qe_rx.is_empty() {
            // wait
        }
        match qe_rx.read() {
            Some(val) => {
                let qe_val = i32::from_le_bytes(val.to_le_bytes());
                // this breaks when the encoder wraps around at...2 billion counts or something.
                if qe_val != last_delta {
                    let going_up = qe_val > last_delta;
                    last_delta = qe_val;
                    match mode {
                        OpMode::Angle => {
                            // handle servo
                            if !going_up {
                                angle += ANGLE_INCREMENT;
                                angle = angle.min(ANGLE_MAX);
                            } else {
                                angle -= ANGLE_INCREMENT;
                                angle = angle.max(ANGLE_MIN);
                            }
                            let mut pulse: f32 = (angle as f32) * (MAX_PULSE - MIN_PULSE) / (ANGLE_MAX as f32 - ANGLE_MIN as f32) + MID_PULSE;
                            pulse = pulse.clamp(MIN_PULSE, MAX_PULSE);
                            servo_cluster.set_pulse(servo1, pulse, false);
                            servo_cluster.load();

                        }
                        OpMode::Brightness => {
                            if going_up {
                                brightness += BRIGHTNESS_INCREMENT;
                                brightness = brightness.min(MAX_PWM);
                            } else {
                                brightness = brightness.saturating_sub(BRIGHTNESS_INCREMENT);
                            }
                            info!("new brightness: {}", brightness);
                            channel.set_duty(brightness);
                        }
                        OpMode::Pixel => {
                            /*
                            if going_up {
                                info!("setting lows");
                                clk.set_low().unwrap();
                                for _i in 0..5 {
                                    data.set_low().unwrap();
                                    count_down.start(shift_delay);
                                    let _ = nb::block!(count_down.wait());
                                    clk.set_high().unwrap();
                                    count_down.start(shift_delay);
                                    let _ = nb::block!(count_down.wait());
                                    clk.set_low().unwrap();
                                }
                            } else {
                                info!("setting highs");
                                clk.set_low().unwrap();
                                for _i in 0..5 {
                                    data.set_high().unwrap();
                                    count_down.start(shift_delay);
                                    let _ = nb::block!(count_down.wait());
                                    clk.set_high().unwrap();
                                    count_down.start(shift_delay);
                                    let _ = nb::block!(count_down.wait());
                                    clk.set_low().unwrap();
                                }
                            } */
                            if going_up {
                                pix_ud_count += 1;
                            } else {
                                pix_ud_count -= 1;
                            }
                            if pix_ud_count.abs() > UD_THRESHOLD as i32 {
                                if pix_ud_count > 0 {
                                    pixel_state <<= 1;
                                    pixel_state = pixel_state.min(0b1_0000);
                                } else {
                                    pixel_state >>= 1;
                                    pixel_state = pixel_state.max(0b0_0001);
                                }
                                // catch if the bit "fell off the edge"
                                let checked_state = if pixel_state == 0 {
                                    0b0_0001
                                } else if pixel_state > 0b1_0000 {
                                    0b1_0000
                                } else {
                                    pixel_state
                                };
                                info!("pixel state: {:b}", checked_state);
                                if let Some(data) = data_pin.as_mut() {
                                    clk.set_low().unwrap();
                                    for i in 0..5 {
                                        if (checked_state >> i) & 1 != 0 {
                                            // low means pixel is on
                                            data.set_low().unwrap();
                                        } else {
                                            data.set_high().unwrap();
                                        }
                                        count_down.start(shift_delay);
                                        let _ = nb::block!(count_down.wait());
                                        clk.set_high().unwrap();
                                        count_down.start(shift_delay);
                                        let _ = nb::block!(count_down.wait());
                                        clk.set_low().unwrap();
                                    }
                                    // otherwise the PWM is off
                                    data.set_high().unwrap();
                                }
                                pix_ud_count = 0;
                            }
                        }
                    }
                }
            },
            _ => info!("QE error"),
        }
        r_interval += 1;
        if r_interval > RELAXATION_INTERVAL {
            // info!("r_interval: {}", r_interval);
            if pix_ud_count > 0 {
                pix_ud_count -= 1;
            } else if pix_ud_count < 0 {
                pix_ud_count += 1;
            }
            r_interval = 0;
        }
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