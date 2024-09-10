## Testing Zenoh with `rustls_post_quantum` CryptoProvider and ROS 2

## Building Zenoh

First, update Rust to the latest version:

```bash
rustup update
```

Then, build Zenoh with all targets in release mode:

```bash
cargo build --release --all-targets
```

## Testing Publisher and Subscriber with TLS and X25519Kyber768Draft00 Post-Quantum Key Exchange

### Generate the certificates

#### Using OpenSSL directly

Use the `generate_certs.sh` script inside the `certs/openssl`folder.

#### Using SROS2

You need to build ROS 2 and source it in your terminal (you can refer for example to this [documentation](https://github.com/fj-blanco/sros2/blob/pq/build_ros2_jazzy.md))

```bash
source <your_ros2_installation>/install/setup.bash
```

```bash
ros2 security create_keystore ./certs/keystore --pq-algorithm dilithium3
ros2 security create_enclave ./certs/keystore /zenoh_router --pq-algorithm dilithium3 --replace
ros2 security create_enclave ./certs/keystore /zenoh_client --pq-algorithm dilithium3 --replace
```

### Running an example

Start the Zenoh router:

```bash
./target/release/zenohd -c ./pq_config/openssl/router.json5
```

In a separate terminal, start a subscriber:

```bash
./target/release/examples/z_sub -c ./pq_config/openssl/client.json5
```

In another terminal, start a publisher:

```bash
./target/release/examples/z_pub -c ./pq_config/openssl/client.json5
```

### Analyzing the Traffic

Use Wireshark to filter the traffic by `tcp.port==7447` and ensure that you see TLSv1.3 traffic.

In the `Client Hello`you should see the following supported groups:

* Unkwnown

This occurs because we have disabled all traditional key exchanges and left only `X25519Kyber768Draft00` (0x6399), which is not recognized by Wireshark.

If you build Zenoh with the default provider instead (not available in this repository—simply clone Zenoh from the official repository), you will see:

* x25519
* secp256r1
* secp384r1

**Note**: These are traditional (non post-quantum) elliptic curves.

