//! localStorage helpers + key constants.
//!
//! All owb-ui localStorage keys share the `owb_ui_` prefix so they can be
//! identified and cleared without touching unrelated site storage.

pub const KEY_THEME: &str = "owb_ui_theme";
pub const KEY_CURVE: &str = "owb_ui_curve";
pub const KEY_TRIM: &str = "owb_ui_trim";
pub const KEY_HH: &str = "owb_ui_hh";
pub const KEY_HOST: &str = "owb_ui_host";
pub const KEY_CONTROL_KEY: &str = "owb_ui_control_key";

fn local_storage() -> Option<web_sys::Storage> { web_sys::window()?.local_storage().ok()? }

pub fn get(key: &str) -> Option<String> { local_storage()?.get_item(key).ok()? }

pub fn set(
    key: &str,
    value: &str,
) {
    if let Some(storage) = local_storage() {
        let _ = storage.set_item(key, value);
    }
}

pub fn get_json<T: serde::de::DeserializeOwned>(key: &str) -> Option<T> {
    let raw = get(key)?;
    serde_json::from_str(&raw).ok()
}

pub fn set_json<T: serde::Serialize>(
    key: &str,
    value: &T,
) {
    if let Ok(s) = serde_json::to_string(value) {
        set(key, &s);
    }
}
