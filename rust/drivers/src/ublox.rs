use embedded_io::{Read, Write, ReadReady};
use embedded_hal::delay::DelayNs;

// ---- ubx config messages, copied from UBLOX_GNSS_Config.ino (checksums included) ----
// format: B5 62 | class id | len lo len hi | payload | ck_a ck_b

const SET_GSA: [u8; 16] = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31]; // turn off GSA sentences
const SET_NAV: [u8; 44] = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x1F]; // airborne <4g nav model
const SET_GSV: [u8; 16] = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39]; // turn off GSV sentences
const SET_DATA_RATE: [u8; 14] = [0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12]; // 10Hz (100ms), M8Q max
const SET_NMEA_4_1: [u8; 28] = [0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0x57]; // nmea version 4.1
const SET_SAT: [u8; 20] = [0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x82, 0x56]; // enable galileo
const SET_VTG: [u8; 16] = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46]; // turn off VTG sentences
const SET_JAM: [u8; 16] = [0xB5, 0x62, 0x06, 0x39, 0x08, 0x00, 0xF3, 0xAC, 0x62, 0xAD, 0x1E, 0x43, 0x00, 0x00, 0x56, 0x45]; // interference thresholds
const SET_GLL: [u8; 16] = [0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B]; // turn off GLL sentences
const SET_BAUD_RATE: [u8; 28] = [0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAF, 0x70]; // 38400 baud

// sent in this order by configure, baud rate must be last (serial speed changes after)
const CONFIG: &[&[u8]] = &[
    &SET_GSA,
    &SET_NAV,
    &SET_GSV,
    &SET_DATA_RATE,
    &SET_NMEA_4_1,
    &SET_SAT,
    &SET_VTG,
    &SET_JAM,
    &SET_GLL,
    &SET_BAUD_RATE,
];

// ---- power save, sent after landing ----
const ENABLE_POWER_SAVE: [u8; 10] = [0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92];
const SET_POWER_SAVE_MODE: [u8; 52] = [0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x0E, 0x90, 0x40, 0x01, 0x20, 0x4E, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x87, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0x67, 0x83];
const SET_DATA_RATE_SLOW: [u8; 14] = [0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x10, 0x27, 0x01, 0x00, 0x01, 0x00, 0x4D, 0xDD]; // once per 10s

const POWER_SAVE: &[&[u8]] = &[
    &ENABLE_POWER_SAVE,
    &SET_POWER_SAVE_MODE,
    &SET_DATA_RATE_SLOW,
];

// ---- ack ----
const UBX_SYNC: [u8; 2] = [0xB5, 0x62];
const ACK_CLASS: u8 = 0x05;
const ACK_ID: u8 = 0x01;   // 0x00 = nak
const ACK_LEN: usize = 10; // B5 62 05 01 02 00 class id ck_a ck_b
const ACK_TIMEOUT_MS: u32 = 1500;
const MAX_RETRIES: u8 = 3;

// ---- nmea ----
const MAX_LINE: usize = 82;      // nmea max sentence length
const GGA_TAG: &str = "GGA";     // match end of $GPGGA / $GNGGA
const MINUTES_PER_DEGREE: f32 = 60.0;

#[derive(Debug)]
pub enum Error<E> {
    Serial(E),
    NoAck { class: u8, id: u8 },
    Nak { class: u8, id: u8 },
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self { Error::Serial(e) }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsReading {
    lat: f32,
    lon: f32,
    alt_m: f32,
    sats: u8,
}

impl GpsReading {
    pub fn lat(&self) -> f32 { self.lat }

    pub fn lon(&self) -> f32 { self.lon }

    pub fn alt_m(&self) -> f32 { self.alt_m }

    pub fn sats(&self) -> u8 { self.sats }
}

pub struct UbloxDriver<S> {
    serial: S,
    line: [u8; MAX_LINE],
    len: usize,
}

impl<S: Read + Write + ReadReady> UbloxDriver<S> {
    pub fn new(serial: S) -> Self {
        Self { serial, line: [0u8; MAX_LINE], len: 0 }
    }

    pub fn configure(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<S::Error>> {
        self.send_messages(CONFIG, delay)
    }

    pub fn poll(&mut self) -> Result<Option<GpsReading>, Error<S::Error>> {
        let mut latest = None;
        let mut chunk = [0u8; 32];

        while self.serial.read_ready()? {
            let n = self.serial.read(&mut chunk)?;
            for &byte in &chunk[..n] {
                if let Some(reading) = self.push_byte(byte) {
                    latest = Some(reading);
                }
            }
        }

        Ok(latest)
    }

    pub fn power_save(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<S::Error>> {
        self.send_messages(POWER_SAVE, delay)
    }

    fn send_messages(&mut self, msgs: &[&[u8]], delay: &mut impl DelayNs) -> Result<(), Error<S::Error>> {
        for msg in msgs {
            self.send_with_retry(msg, delay)?;
        }
        Ok(())
    }

    fn send_with_retry(&mut self, msg: &[u8], delay: &mut impl DelayNs) -> Result<(), Error<S::Error>> {
        let mut result = Err(Error::NoAck { class: msg[2], id: msg[3] });

        for _ in 0..MAX_RETRIES {
            self.serial.write_all(msg)?;
            self.serial.flush()?;
            result = self.wait_ack(msg, delay);
            match result {
                Ok(()) | Err(Error::Serial(_)) => return result,
                _ => continue, // nak or timeout
            }
        }

        result // only reached on nak or timeout
    }

    fn wait_ack(&mut self, msg: &[u8], delay: &mut impl DelayNs) -> Result<(), Error<S::Error>> {
        let class = msg[2];
        let id = msg[3];
        let header = [UBX_SYNC[0], UBX_SYNC[1], ACK_CLASS];
        let mut ack = [0u8; ACK_LEN];
        let mut len = 0;

        for _ in 0..ACK_TIMEOUT_MS {
            while self.serial.read_ready()? {
                let mut byte = [0u8; 1];
                self.serial.read(&mut byte)?;
                let byte = byte[0];

                if len < header.len() {
                    if byte == header[len] {
                        ack[len] = byte;
                        len += 1;
                    } else if byte == UBX_SYNC[0] {
                        ack[0] = byte;
                        len = 1;
                    } else {
                        len = 0;
                    }
                } else {
                    ack[len] = byte;
                    len += 1;
                }

                if len == ACK_LEN {
                    return check_ack(&ack, class, id);
                }
            }
            delay.delay_ms(1);
        }
        Err(Error::NoAck { class, id })
    }

    // builds line by byte, if we see last byte parse and return the reading
    fn push_byte(&mut self, byte: u8) -> Option<GpsReading> {
        match byte {
            b'$' => {
                // start of sentence
                self.line[0] = byte;
                self.len = 1;
                None
            }
            b'\n' => {
                // end of sentence, \r gets trimmed in parse_gga
                let reading = core::str::from_utf8(&self.line[..self.len]).ok().and_then(parse_gga);
                self.len = 0;
                reading
            }
            _ => {
                if self.len < MAX_LINE {
                    self.line[self.len] = byte;
                    self.len += 1;
                } else {
                    self.len = 0;
                }
                None
            }
        }
    }
}

// ack = B5 62 05 01 02 00 class id ck_a ck_b
fn check_ack<E>(ack: &[u8; ACK_LEN], class: u8, id: u8) -> Result<(), Error<E>> {
    if ack[3] != ACK_ID {
        return Err(Error::Nak { class, id });
    }

    let (ck_a, ck_b) = ubx_checksum(&ack[2..8]);
    if ack[6] != class || ack[7] != id || ck_a != ack[8] || ck_b != ack[9] {
        return Err(Error::NoAck { class, id });
    }

    Ok(())
}

// ubx fletcher checksum, runs over class/id/len/payload
fn ubx_checksum(bytes: &[u8]) -> (u8, u8) {
    bytes.iter().fold((0u8, 0u8), |(a, b), &byte| {
        let a = a.wrapping_add(byte);
        (a, b.wrapping_add(a))
    })
}

fn parse_gga(line: &str) -> Option<GpsReading> {
    let (body, checksum) = line.trim_end().strip_prefix('$')?.split_once('*')?;

    let body_sum = body.bytes().fold(0u8, |acc, b| acc ^ b);

    if body_sum != u8::from_str_radix(checksum, 16).ok()? {
        return None;
    }

    let mut fields = body.split(',');

    let tag = fields.next()?;
    if !tag.ends_with(GGA_TAG) {
        return None;
    }

    let _time = fields.next()?;
    let lat = fields.next()?;
    let north_south = fields.next()?;
    let lon = fields.next()?;
    let east_west = fields.next()?;
    let quality = fields.next()?;
    let sats = fields.next()?;
    let _hdop = fields.next()?;
    let alt = fields.next()?;

    if quality == "0" {
        return None;
    }

    let mut lat = ddmm_to_degrees(lat, 2)?;
    if north_south == "S" {
        lat = -lat;
    }

    let mut lon = ddmm_to_degrees(lon, 3)?;
    if east_west == "W" {
        lon = -lon;
    }

    Some(GpsReading {
        lat,
        lon,
        alt_m: alt.parse().ok()?,
        sats: sats.parse().ok()?,
    })
}

// "4807.038" with 2 degree digits -> 48 + 7.038 / 60
fn ddmm_to_degrees(field: &str, deg_digits: usize) -> Option<f32> {
    let degrees: f32 = field.get(..deg_digits)?.parse().ok()?;
    let minutes: f32 = field.get(deg_digits..)?.parse().ok()?;
    Some(degrees + minutes / MINUTES_PER_DEGREE)
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_io::ErrorKind;
    use std::collections::VecDeque;

    // fake uart: rx is what the gps "sends", tx records what we wrote
    struct FakeSerial {
        rx: VecDeque<u8>,
        tx: Vec<u8>,
    }

    impl FakeSerial {
        fn new(rx: &[u8]) -> Self {
            Self { rx: rx.iter().copied().collect(), tx: Vec::new() }
        }
    }

    impl embedded_io::ErrorType for FakeSerial {
        type Error = ErrorKind;
    }

    impl Read for FakeSerial {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, ErrorKind> {
            let n = buf.len().min(self.rx.len());
            for b in &mut buf[..n] {
                *b = self.rx.pop_front().unwrap();
            }
            Ok(n)
        }
    }

    impl ReadReady for FakeSerial {
        fn read_ready(&mut self) -> Result<bool, ErrorKind> {
            Ok(!self.rx.is_empty())
        }
    }

    impl Write for FakeSerial {
        fn write(&mut self, buf: &[u8]) -> Result<usize, ErrorKind> {
            self.tx.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> Result<(), ErrorKind> { Ok(()) }
    }

    // wraps a body in $...*XX\r\n with the right checksum
    fn nmea(body: &str) -> String {
        let sum = body.bytes().fold(0u8, |acc, b| acc ^ b);
        format!("${}*{:02X}\r\n", body, sum)
    }

    // builds an ack (ack_id 0x01) or nak (0x00) for a message
    fn ack_for(msg: &[u8], ack_id: u8) -> Vec<u8> {
        let mut ack = vec![0xB5, 0x62, ACK_CLASS, ack_id, 0x02, 0x00, msg[2], msg[3]];
        let (a, b) = ubx_checksum(&ack[2..8]);
        ack.extend([a, b]);
        ack
    }

    const GGA: &str = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";

    #[test]
    fn parse_gga_valid() {
        let r = parse_gga(&nmea(GGA)).unwrap();

        assert!((r.lat() - 48.1173).abs() < 1e-4);
        assert!((r.lon() - 11.5167).abs() < 1e-4);
        assert_eq!(r.alt_m(), 545.4);
        assert_eq!(r.sats(), 8);
    }

    #[test]
    fn parse_gga_south_west_negative() {
        let r = parse_gga(&nmea("GNGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,")).unwrap();

        assert!(r.lat() < 0.0 && r.lon() < 0.0);
    }

    #[test]
    fn parse_gga_rejects_bad_checksum() {
        assert_eq!(parse_gga(&format!("${}*00", GGA)), None);
    }

    #[test]
    fn parse_gga_rejects_no_fix() {
        assert_eq!(parse_gga(&nmea("GPGGA,123519,,,,,0,00,,,M,,M,,")), None);
    }

    #[test]
    fn parse_gga_ignores_other_sentences() {
        assert_eq!(parse_gga(&nmea("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W")), None);
    }

    #[test]
    fn poll_skips_junk_and_returns_reading() {
        let rx = format!("garbage{}{}", nmea("GPGSA,A,3"), nmea(GGA));
        let mut gps = UbloxDriver::new(FakeSerial::new(rx.as_bytes()));

        assert_eq!(gps.poll().unwrap().unwrap().sats(), 8);
    }

    #[test]
    fn poll_returns_none_when_empty() {
        let mut gps = UbloxDriver::new(FakeSerial::new(&[]));

        assert_eq!(gps.poll().unwrap(), None);
    }

    #[test]
    fn configure_sends_all_messages() {
        let rx: Vec<u8> = CONFIG.iter().flat_map(|m| ack_for(m, ACK_ID)).collect();
        let mut gps = UbloxDriver::new(FakeSerial::new(&rx));

        gps.configure(&mut NoopDelay).unwrap();

        assert_eq!(gps.serial.tx, CONFIG.concat());
    }

    #[test]
    fn configure_retries_after_nak() {
        let mut rx = ack_for(CONFIG[0], 0x00);
        rx.extend(CONFIG.iter().flat_map(|m| ack_for(m, ACK_ID)));
        let mut gps = UbloxDriver::new(FakeSerial::new(&rx));

        gps.configure(&mut NoopDelay).unwrap();

        // first message sent twice
        assert_eq!(&gps.serial.tx[..2 * CONFIG[0].len()], [CONFIG[0], CONFIG[0]].concat());
    }

    #[test]
    fn configure_gives_up_after_retries() {
        let mut gps = UbloxDriver::new(FakeSerial::new(&[]));

        let result = gps.configure(&mut NoopDelay);

        assert!(matches!(result, Err(Error::NoAck { class, id }) if class == CONFIG[0][2] && id == CONFIG[0][3]));
        assert_eq!(gps.serial.tx, CONFIG[0].repeat(MAX_RETRIES as usize));
    }
}