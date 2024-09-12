#!/bin/bash

# Generate CA key and certificate
openssl req -x509 -newkey rsa:4096 -days 365 -nodes -keyout ca.key -out ca.pem -subj "/CN=Zenoh Test CA" -sha256 -extensions v3_ca -config <(
  echo "[v3_ca]"
  echo "basicConstraints = critical, CA:TRUE"
  echo "keyUsage = critical, digitalSignature, keyCertSign, cRLSign"
)

# Generate server key
openssl genrsa -out server.key 2048

# Generate server CSR
openssl req -new -key server.key -out server.csr -subj "/CN=localhost"

# Sign the server certificate with our CA
openssl x509 -req -in server.csr -CA ca.pem -CAkey ca.key -CAcreateserial -out server.pem -days 365 -sha256 -extfile <(
  echo "subjectAltName=DNS:localhost,IP:127.0.0.1"
  echo "keyUsage=critical,digitalSignature,keyEncipherment"
  echo "extendedKeyUsage=serverAuth"
)

# Create the full chain
cat server.pem ca.pem > server.fullchain.pem

# Generate client key
openssl genrsa -out client.key 2048

# Generate client CSR
openssl req -new -key client.key -out client.csr -subj "/CN=Zenoh Client"

# Sign the client certificate with our CA
openssl x509 -req -in client.csr -CA ca.pem -CAkey ca.key -CAcreateserial -out client.pem -days 365 -sha256 -extfile <(
  echo "subjectAltName=DNS:localhost,IP:127.0.0.1"
  echo "keyUsage=critical,digitalSignature,keyEncipherment"
  echo "extendedKeyUsage=clientAuth"
)

# Clean up temporary files
rm *.csr *.srl

echo "Certificates generated in the current directory"
