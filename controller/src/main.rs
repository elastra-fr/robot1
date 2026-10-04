use controller::{app::Runtime, config::Config};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    Runtime::new(Config::from_env()?).run()
}
