#![no_std]
#![no_main]

use defmt_rtt as _;

#[cfg(test)]
#[embedded_test::tests]
mod unit_tests {
    use embassy_stm32::Peripherals;

    #[init]
    async fn init() -> Peripherals {
        embassy_stm32::init(Default::default())
    }

    #[test]
    async fn takes_state(_state: Peripherals) {
        assert!(true)
    }

    #[test]
    fn defmt() {
        defmt::info!("Hello, defmt!");
        assert!(false)
    }

    #[test]
    fn it_fails1() {
        assert!(false)
    }

    #[test]
    fn it_fails2() -> Result<(), &'static str> {
        Err("It failed because ...")
    }

    #[test]
    #[should_panic]
    fn it_passes() {
        assert!(false)
    }

    #[test]
    #[should_panic]
    fn it_fails3() {}

    #[test]
    #[timeout(1)]
    fn it_timeouts() {
        loop {}
    }
}
