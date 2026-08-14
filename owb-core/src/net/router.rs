//! Embedded HTTP route composition.

use picoserve::{
    routing::{PathRouter, RequestHandlerFunction},
    Router,
};

use crate::net::{
    server::{frontend_index_response, API_SCHEMA_PAGE, HEALTHZ_PAGE, MINIMAL_INDEX_PAGE},
    session::{now_millis, now_secs, SessionManager},
    websocket::{WebSocket, WsConnectionParams},
};

fn attach_ws_route<P>(router: Router<P>) -> Router<impl PathRouter>
where
    P: PathRouter,
{
    router.route(
        "/ws",
        picoserve::routing::get(|params: WsConnectionParams| async move {
            let session_id = params.query.session;
            tracing::info!("New WebSocket connection with session id: {}", session_id);
            // Authenticate before reserving the control lease so an unauthorized
            // client can neither drive motors nor deny service by squatting the
            // single-controller lock.
            let authorized = crate::net::auth::verify_control_secret(&params.query.key).await;
            if !authorized {
                tracing::warn!(session_id = %session_id, "websocket rejected; invalid control secret");
            } else if !crate::net::auth::control_secret_required().await {
                tracing::warn!("control secret not configured; control surface is UNAUTHENTICATED");
            }
            let now = now_secs();
            let lease_id = now_millis();
            let accepted = authorized
                && SessionManager::try_acquire_session(session_id.clone(), now, lease_id).await;
            if authorized && !accepted {
                tracing::warn!(session_id = %session_id, "websocket rejected; controller already connected");
            }
            params
                .upgrade
                .on_upgrade(WebSocket::new(session_id.clone(), lease_id, accepted, authorized))
                .with_protocol("messages")
        }),
    )
}

pub fn static_router() -> Router<impl PathRouter> {
    Router::new()
        .route(
            "/",
            picoserve::routing::get(|| async { frontend_index_response() }),
        )
        .route(
            "/index.html",
            picoserve::routing::get(|| async { frontend_index_response() }),
        )
        .route(
            "/setup",
            picoserve::routing::get(|| async { frontend_index_response() }),
        )
        .route(
            "/setup/",
            picoserve::routing::get(|| async { frontend_index_response() }),
        )
        .route(
            "/healthz",
            picoserve::routing::get(|| async { HEALTHZ_PAGE }),
        )
}

/// Build a control-only router exposing just the `/ws` endpoint.
pub fn control_router() -> Router<impl PathRouter> { attach_ws_route(Router::new()) }

/// Build the default HTTP router with static assets and the `/ws` endpoint.
pub fn base_router() -> Router<impl PathRouter> {
    attach_ws_route(static_router()).route(
        "/api/schema",
        picoserve::routing::get(|| async { API_SCHEMA_PAGE }),
    )
}

/// Build the compact HIL/test router for constrained bring-up.
pub fn minimal_router() -> Router<impl PathRouter> {
    attach_ws_route(
        Router::new()
            .route(
                "/",
                picoserve::routing::get(|| async { MINIMAL_INDEX_PAGE }),
            )
            .route(
                "/index.html",
                picoserve::routing::get(|| async { MINIMAL_INDEX_PAGE }),
            )
            .route(
                "/motor-test",
                picoserve::routing::get(|| async { MINIMAL_INDEX_PAGE }),
            )
            .route(
                "/motor-test/",
                picoserve::routing::get(|| async { MINIMAL_INDEX_PAGE }),
            )
            .route(
                "/healthz",
                picoserve::routing::get(|| async { HEALTHZ_PAGE }),
            ),
    )
    .route(
        "/api/schema",
        picoserve::routing::get(|| async { API_SCHEMA_PAGE }),
    )
}

/// Build the default HTTP router plus the shared `/camera` test page.
pub fn base_router_with_camera_page() -> Router<impl PathRouter> {
    attach_ws_route(static_router())
        .route(
            "/api/schema",
            picoserve::routing::get(|| async { API_SCHEMA_PAGE }),
        )
        .route(
            "/camera",
            picoserve::routing::get(|| async { frontend_index_response() }),
        )
        .route(
            "/camera/",
            picoserve::routing::get(|| async { frontend_index_response() }),
        )
}

/// Build the dedicated media router for the standard `/camera/stream`
/// endpoint while keeping the handler implementation board-specific.
pub fn camera_stream_router<H, Sig>(handler: H) -> Router<impl PathRouter>
where
    H: RequestHandlerFunction<(), (), Sig>,
{
    Router::new().route("/camera/stream", picoserve::routing::get(handler))
}
