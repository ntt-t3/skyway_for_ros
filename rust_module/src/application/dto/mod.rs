pub(crate) mod request;
pub(crate) mod response;

pub(crate) trait Command {
    fn command(&self) -> String;
}
