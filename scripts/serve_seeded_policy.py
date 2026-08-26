#!/usr/bin/env python3
"""Serve OpenPI with deterministic per-episode, per-replan sampling."""

from __future__ import annotations

import asyncio
import dataclasses
import hashlib
import http
import logging
import time
import traceback
from typing import Any

import jax
import jax.numpy as jnp
import numpy as np
from openpi_client import msgpack_numpy
from openpi.policies import policy_config
from openpi.training import config as training_config
import tyro
import websockets.asyncio.server as websocket_server
import websockets.frames


SAPS_PROTOCOL_VERSION = 1


@dataclasses.dataclass
class Args:
    config_name: str = "pi05_libero"
    checkpoint_dir: str = (
        "gs://openpi-assets/checkpoints/pi05_libero"
    )
    host: str = "0.0.0.0"
    port: int = 8000


class SeededWebsocketPolicyServer:
    def __init__(
        self,
        *,
        policy: Any,
        host: str,
        port: int,
        config_name: str,
        checkpoint_dir: str,
    ) -> None:
        self._policy = policy
        self._host = host
        self._port = port

        model = getattr(policy, "_model", None)

        if model is None:
            raise RuntimeError(
                "Could not access the OpenPI model for sampling dimensions."
            )

        self._action_horizon = int(model.action_horizon)
        self._action_dim = int(model.action_dim)

        self._metadata = dict(policy.metadata)
        self._metadata["saps_seeded_sampling"] = {
            "protocol_version": SAPS_PROTOCOL_VERSION,
            "action_horizon": self._action_horizon,
            "latent_action_dim": self._action_dim,
            "policy_config_name": config_name,
            "policy_checkpoint": checkpoint_dir,
        }

    def serve_forever(self) -> None:
        asyncio.run(self.run())

    async def run(self) -> None:
        async with websocket_server.serve(
            self._handler,
            self._host,
            self._port,
            compression=None,
            max_size=None,
            process_request=_health_check,
        ) as server:
            await server.serve_forever()

    def _make_noise(
        self,
        *,
        policy_episode_seed: int,
        replan_index: int,
    ) -> np.ndarray:
        if not 0 <= policy_episode_seed <= 0x7FFFFFFF:
            raise ValueError(
                "policy_episode_seed must be within [0, 2^31 - 1]."
            )

        if replan_index < 0:
            raise ValueError("replan_index must be non-negative.")

        episode_key = jax.random.key(policy_episode_seed)
        replan_key = jax.random.fold_in(
            episode_key,
            replan_index,
        )

        noise = jax.random.normal(
            replan_key,
            (
                self._action_horizon,
                self._action_dim,
            ),
            dtype=jnp.float32,
        )

        return np.asarray(noise, dtype=np.float32)

    async def _handler(
        self,
        websocket: websocket_server.ServerConnection,
    ) -> None:
        logging.info(
            "Connection from %s opened",
            websocket.remote_address,
        )

        packer = msgpack_numpy.Packer()
        await websocket.send(packer.pack(self._metadata))

        previous_total_time = None

        while True:
            try:
                start_time = time.monotonic()
                request = msgpack_numpy.unpackb(
                    await websocket.recv()
                )

                sampling_metadata = None

                if (
                    isinstance(request, dict)
                    and request.get(
                        "__saps_protocol_version__"
                    )
                    == SAPS_PROTOCOL_VERSION
                ):
                    observation = request["observation"]
                    policy_episode_seed = int(
                        request["policy_episode_seed"]
                    )
                    replan_index = int(
                        request["replan_index"]
                    )

                    noise = self._make_noise(
                        policy_episode_seed=policy_episode_seed,
                        replan_index=replan_index,
                    )

                    noise_sha256 = hashlib.sha256(
                        noise.tobytes()
                    ).hexdigest()

                    inference_start = time.monotonic()
                    result = self._policy.infer(
                        observation,
                        noise=noise,
                    )
                    inference_seconds = (
                        time.monotonic() - inference_start
                    )

                    sampling_metadata = {
                        "protocol_version": (
                            SAPS_PROTOCOL_VERSION
                        ),
                        "policy_episode_seed": (
                            policy_episode_seed
                        ),
                        "replan_index": replan_index,
                        "noise_sha256": noise_sha256,
                        "action_horizon": (
                            self._action_horizon
                        ),
                        "latent_action_dim": (
                            self._action_dim
                        ),
                    }
                else:
                    # Backward-compatible ordinary OpenPI request.
                    inference_start = time.monotonic()
                    result = self._policy.infer(request)
                    inference_seconds = (
                        time.monotonic() - inference_start
                    )

                result["server_timing"] = {
                    "infer_ms": inference_seconds * 1000.0,
                }

                if previous_total_time is not None:
                    result["server_timing"][
                        "prev_total_ms"
                    ] = previous_total_time * 1000.0

                if sampling_metadata is not None:
                    result["saps_sampling"] = (
                        sampling_metadata
                    )

                await websocket.send(packer.pack(result))
                previous_total_time = (
                    time.monotonic() - start_time
                )

            except websockets.ConnectionClosed:
                logging.info(
                    "Connection from %s closed",
                    websocket.remote_address,
                )
                break

            except Exception:
                await websocket.send(
                    traceback.format_exc()
                )
                await websocket.close(
                    code=(
                        websockets.frames.CloseCode
                        .INTERNAL_ERROR
                    ),
                    reason=(
                        "Internal error; traceback was "
                        "sent in the previous frame."
                    ),
                )
                raise


def _health_check(
    connection: websocket_server.ServerConnection,
    request: websocket_server.Request,
) -> websocket_server.Response | None:
    if request.path == "/healthz":
        return connection.respond(
            http.HTTPStatus.OK,
            "OK\n",
        )

    return None


def main(args: Args) -> None:
    config = training_config.get_config(
        args.config_name
    )

    policy = policy_config.create_trained_policy(
        config,
        args.checkpoint_dir,
    )

    logging.info(
        "Creating deterministic SAPS policy server "
        "on %s:%d",
        args.host,
        args.port,
    )

    server = SeededWebsocketPolicyServer(
        policy=policy,
        host=args.host,
        port=args.port,
        config_name=args.config_name,
        checkpoint_dir=args.checkpoint_dir,
    )
    server.serve_forever()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
        force=True,
    )
    main(tyro.cli(Args))
