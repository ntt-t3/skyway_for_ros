pub(crate) mod peer;

use shaku::{Component, Interface};

#[cfg(test)]
use mockall::automock;

#[cfg_attr(test, automock)]
pub(crate) trait Service: Interface {}

#[derive(Component)]
#[shaku(interface = Service)]
pub(crate) struct ServiceImpl {}

impl Service for ServiceImpl {}
