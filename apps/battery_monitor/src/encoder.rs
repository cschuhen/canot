//! Encoder input handling (button + rotary encoder).
//!
//! Debounces inputs, tracks quadrature state, and sends HID events.

use crate::application::{HumanEvent, MainEvent};
use crate::error::Error;
use embassy_futures::select::{select3, Either3};
use embassy_time::{Duration, Instant};
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use j1939_async::error::SendError;

use crate::EncoderArgs;

#[derive(PartialEq)]
enum EncoderState {
    A,
    B,
    Idle,
}

/// Run the encoder loop: debounce inputs, track quadrature rotation, and handle button presses.
pub async fn run_encoder(args: &mut EncoderArgs) {
    let EncoderArgs(pins, sender, error_sender) = args;
    let [button, enc_a, enc_b] = pins;

    // Debounce the inputs
    let mut enc_a = async_debounce::Debouncer::new(enc_a, Duration::from_micros(10));
    let mut enc_b = async_debounce::Debouncer::new(enc_b, Duration::from_micros(10));
    let mut button = async_debounce::Debouncer::new(button, Duration::from_micros(100));

    let mut press_time: Option<Instant> = None;
    let mut encoder_state: EncoderState = EncoderState::Idle;

    loop {
        let res: Result<(), Error> = match select3(
            enc_a.wait_for_any_edge(),
            enc_b.wait_for_any_edge(),
            button.wait_for_any_edge(),
        )
        .await
        {
            // For Encoder A and B. Just look for A,B raising edges or B,A raising edges. Any falling edge, resets state to idle.
            Either3::First(_) => match enc_a.is_high() {
                Ok(true) => {
                    if encoder_state == EncoderState::B {
                        encoder_state = EncoderState::Idle;
                        sender
                            .send(MainEvent::HID(HumanEvent::EncoderAntiClockwise))
                            .await;
                        Ok(())
                    } else {
                        encoder_state = EncoderState::A;
                        Ok(())
                    }
                }
                Ok(false) => {
                    encoder_state = EncoderState::Idle;
                    Ok(())
                }
                Err(_) => {
                    encoder_state = EncoderState::Idle;
                    Ok(())
                }
            },
            Either3::Second(_) => match enc_b.is_high() {
                Ok(true) => {
                    if encoder_state == EncoderState::A {
                        encoder_state = EncoderState::Idle;
                        sender
                            .send(MainEvent::HID(HumanEvent::EncoderClockwise))
                            .await;
                        Ok(())
                    } else {
                        encoder_state = EncoderState::B;
                        Ok(())
                    }
                }
                Ok(false) => {
                    encoder_state = EncoderState::Idle;
                    Ok(())
                }
                Err(_) => {
                    encoder_state = EncoderState::Idle;
                    Ok(())
                }
            },
            Either3::Third(_) => match button.is_high() {
                Ok(true) => {
                    press_time = Some(Instant::now());
                    sender
                        .send(MainEvent::HID(HumanEvent::EncoderButtonPressed))
                        .await;
                    Ok(())
                }
                Ok(false) => {
                    if press_time.is_none() {
                        // Spurious?
                        Ok(())
                    } else {
                        let duration = Instant::now() - press_time.unwrap();
                        sender
                            .send(MainEvent::HID(HumanEvent::EncoderButtonReleased(duration)))
                            .await;
                        Ok(())
                    }
                }
                Err(_) => {
                    encoder_state = EncoderState::Idle;
                    Ok(())
                }
            },
        };
        match res {
            Ok(_) => {}
            Err(e) => {
                error_sender.send(&e);
            }
        }
    }
}
