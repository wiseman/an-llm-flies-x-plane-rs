//! Agent-authored flight plans: the IR, the Python front end that builds
//! it, reference resolution and the hardcoded place lists.

pub mod compile;
pub mod cond;
pub mod fixtures;
pub mod gazetteer;
pub mod ir;
pub mod preview;
pub mod python;
pub mod resolve;
pub mod session;
