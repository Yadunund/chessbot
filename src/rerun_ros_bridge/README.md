# rerun_ros_bridge

A generic ROS 2 to [Rerun](https://rerun.io) bridge, as a composable node.

- Load it into the same container as your image publishers with `use_intra_process_comms: true`, so frames reach it by pointer instead of being serialised.
- It serves a Rerun gRPC stream (`rerun+http://<host>:<grpc_port>/proxy`) that the stock native or web viewer connects to, and can archive the same data to an `.rrd` file.
- Everything it logs is chosen by parameters. It has no application-specific code, so it can be reused across projects.

## Supported topics

| ROS type | Rerun archetype | Notes |
|---|---|---|
| `sensor_msgs/Image` | `Image` / `DepthImage` | rgb8, bgr8, rgba8, bgra8, mono8, mono16, 16UC1 (mm), 32FC1 (m); row padding and big-endian handled; optional downscale |
| `sensor_msgs/CompressedImage` | `EncodedImage` | JPEG/PNG passed through without decoding |
| `sensor_msgs/JointState` | `Scalars` per joint | |

Each image topic is rate-limited (`image_rate_hz`) so a fast camera does not flood the stream.

## Viewing

```bash
rerun --serve-web rerun+http://<host>:9876/proxy --web-viewer-port 9090   # browser at http://<host>:9090
rerun --connect rerun+http://<host>:9876/proxy                            # native viewer
```

See `config/example.yaml` for all parameters.
