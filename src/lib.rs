extern crate modbus;

use modbus::tcp;
use modbus::Client;

static ENCODER_POS_1_REG: u16 = 4;
static ENCODER_POS_2_REG: u16 = 5;
static MAX_32_BIT: u64 = 65536;

pub fn new() {
    println!("New applied device.");

    let mut client = tcp::Transport::new("192.168.0.200").unwrap();

    for (n, i) in client
        .read_holding_registers(0, 56)
        .expect("IO Error")
        .iter()
        .enumerate()
    {
        println!("Register {}: {}", n, i);
    }

    println!("Done reading registers.");
    println!("Encoder position: {}", get_encoder_count());
}

pub fn get_encoder_count() -> u64 {
    let mut client = tcp::Transport::new("192.168.0.200").unwrap();

    let x: u16 = *client
        .read_holding_registers(ENCODER_POS_1_REG, 1)
        .expect("IO Error")
        .first()
        .unwrap();

    let y: u16 = *client
        .read_holding_registers(ENCODER_POS_2_REG, 1)
        .expect("IO Error")
        .first()
        .unwrap();

    let encoder_position: u64 = y as u64 + (x as u64 * MAX_32_BIT);

    encoder_position
}
