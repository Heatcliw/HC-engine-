#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TypeLogs {
    Debug, 
    Info,
    Error,
    Warn,
    Trace,
    None
}
impl TypeLogs {
    pub fn as_str(self) -> &'static str {
        match self {
            TypeLogs::Debug => "debug",
            TypeLogs::Error => "error",
            TypeLogs::Info => "info",
            TypeLogs::Warn => "warn",
            TypeLogs::Trace => "trace",
            TypeLogs::None => "off",
        }
    }    
}

pub fn build_logger(Level: TypeLogs){
    let env =env_logger::Env::default().default_filter_or(Level.as_str());
    let _ = env_logger::Builder::from_env(env).init();
}
