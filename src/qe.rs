use pimoroni_servo2040::hal::pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO, StateMachine, Rx, Tx};
use rp2040_hal::gpio::FunctionConfig;
use rp2040_hal::pio::{PIOBuilder, Stopped};

pub fn quadrature_encoder<P, SM>(
    pio: &mut PIO<P>,
    sm: UninitStateMachine<(P, SM)>,
    base: u8,
) -> (StateMachine<(P, SM), Stopped>, Rx<(P, SM)>, Tx<(P, SM)>)
where
    P: PIOExt + FunctionConfig,
    SM: StateMachineIndex,
{
    let qe_prog = pio_proc::pio_asm!(
        // ; this code must be loaded into address 0, but at 29 instructions, it probably
        // ; wouldn't be able to share space with other programs anyway
        ".origin 0",
        // ; the code works by running a loop that continuously shifts the 2 phase pins into
        // ; ISR and looks at the lower 4 bits to do a computed jump to an instruction that
        // ; does the proper "do nothing" | "increment" | "decrement" action for that pin
        // ; state change (or no change)
        //
        // ; ISR holds the last state of the 2 pins during most of the code. The Y register
        // ; keeps the current encoder count and is incremented / decremented according to
        // ; the steps sampled
        //
        // ; writing any non zero value to the TX FIFO makes the state machine push the
        // ; current count to RX FIFO between 6 to 18 clocks afterwards. The worst case
        // ; sampling loop takes 14 cycles, so this program is able to read step rates up
        // ; to sysclk / 14  (e.g., sysclk 125MHz, max step rate = 8.9 Msteps/sec)

        // ; 00 state
        "    jmp update     ", // ; read 00
        "    jmp decrement	", // ; read 01
        "    jmp increment	", // ; read 10
        "    jmp update	    ", // ; read 11

        // ; 01 state
        "    jmp increment	", // ; read 00
        "    jmp update	    ", // ; read 01
        "    jmp update	    ", // ; read 10
        "    jmp decrement	", // ; read 11

        // ; 10 state
        "    jmp decrement	", // ; read 00
        "    jmp update	    ", // ; read 01
        "    jmp update	    ", // ; read 10
        "    jmp increment	", // ; read 11

        // ; to reduce code size, the last 2 states are implemented in place and become the
        // ; target for the other jumps

        // ; 11 state
        "    jmp update	    ", // ; read 00
        "    jmp increment	", // ; read 01
        "decrement:",
            // ; note: the target of this instruction must be the next address, so that
            // ; the effect of the instruction does not depend on the value of Y. The
            // ; same is true for the "JMP X--" below. Basically "JMP Y--, <next addr>"
            // ; is just a pure "decrement Y" instruction, with no other side effects
        "    jmp y--, update", 	//; read 10

        //    ; this is where the main loop starts
        ".wrap_target",
        "update:",
            // ; we start by checking the TX FIFO to see if the main code is asking for
            // ; the current count after the PULL noblock, OSR will have either 0 if
            // ; there was nothing or the value that was there
        "    set x, 0",
        "    pull noblock",

            // ; since there are not many free registers, and PULL is done into OSR, we
            // ; have to do some juggling to avoid losing the state information and
            // ; still place the values where we need them
        "    mov x, osr",
        "    mov osr, isr",

            // ; the main code did not ask for the count, so just go to "sample_pins"
        "    jmp !x, sample_pins",

            // ; if it did ask for the count, then we push it
        "    mov isr, y", //	; we trash ISR, but we already have a copy in OSR
        "    push",

        "sample_pins:",
            // ; we shift into ISR the last state of the 2 input pins (now in OSR) and
            // ; the new state of the 2 pins, thus producing the 4 bit target for the
            // ; computed jump into the correct action for this state
        "    mov isr, null",
        "    in osr, 2",
        "    in pins, 2",
        "    mov pc, isr",

            // ; the PIO does not have a increment instruction, so to do that we do a
            // ; negate, decrement, negate sequence
        "increment:",
        "    mov x, !y",
        "    jmp x--, increment_cont",
        "increment_cont:",
        "    mov y, !x",
        ".wrap	", // ; the .wrap here avoids one jump instruction and saves a cycle too
    );
    let installed = pio.install(&qe_prog.program).unwrap();
    PIOBuilder::from_program(installed)
        .in_pin_base(base)
        .jmp_pin(base)
        .in_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
        .autopull(false)
        .pull_threshold(32)
        .buffers(rp2040_hal::pio::Buffers::RxTx)
        .build(sm)

}