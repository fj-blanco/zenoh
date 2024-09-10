#!/bin/bash

# Generate CA key and certificate
openssl req -x509 -newkey rsa:4096 -days 365 -nodes -keyout ca.key -out ca.pem -subj "/CN=Zenoh Test CA" -sha256

# Generate server key
openssl genrsa -out server.key 2048

# Generate server CSR
openssl req -new -key server.key -out server.csr -subj "/CN=localhost"

# Sign the server certificate with our CA
openssl x509 -req -in server.csr -CA ca.pem -CAkey ca.key -CAcreateserial -out server.pem -days 365 -sha256 -extfile <(printf "subjectAltName=DNS:localhost")

# Create the full chain
cat server.pem ca.pem > server.fullchain.pem

# Generate client key
openssl genrsa -out client.key 2048

# Generate client CSR
openssl req -new -key client.key -out client.csr -subj "/CN=Zenoh Client"

# Sign the client certificate with our CA
openssl x509 -req -in client.csr -CA ca.pem -CAkey ca.key -CAcreateserial -out client.pem -days 365 -sha256

# Clean up temporary files
rm *.csr *.srl

echo "Certificates generated in ./certs directory"