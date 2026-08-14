use leptos::prelude::*;
mod app;
mod components;
mod storage;
mod throttle;
mod transport;

#[wasm_bindgen::prelude::wasm_bindgen(start)]
pub fn main() {
    console_error_panic_hook::set_once();
    mount_to_body(app::App);
}
