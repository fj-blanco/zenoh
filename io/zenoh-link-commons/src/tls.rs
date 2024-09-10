use alloc::vec::Vec;

use rustls::{
    client::{
        danger::{ServerCertVerified, ServerCertVerifier},
        verify_server_cert_signed_by_trust_anchor,
    },
    crypto::{verify_tls12_signature, verify_tls13_signature},
    pki_types::{CertificateDer, ServerName, UnixTime},
    server::ParsedCertificate,
    RootCertStore,
};
use webpki::ALL_VERIFICATION_ALGS;
use rustls::crypto::CryptoProvider;

impl ServerCertVerifier for WebPkiVerifierAnyServerName {
    /// Will verify the certificate is valid in the following ways:
    /// - Signed by a  trusted `RootCertStore` CA
    /// - Not Expired
    fn verify_server_cert(
        &self,
        end_entity: &CertificateDer<'_>,
        intermediates: &[CertificateDer<'_>],
        _server_name: &ServerName<'_>,
        _ocsp_response: &[u8],
        now: UnixTime,
    ) -> Result<ServerCertVerified, rustls::Error> {
        let cert = ParsedCertificate::try_from(end_entity)?;
        verify_server_cert_signed_by_trust_anchor(
            &cert,
            &self.roots,
            intermediates,
            now,
            ALL_VERIFICATION_ALGS,
        )?;
        Ok(ServerCertVerified::assertion())
    }

    fn verify_tls12_signature(
        &self,
        message: &[u8],
        cert: &CertificateDer<'_>,
        dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        let default_provider = rustls::crypto::ring::default_provider();
        let algorithms = self.crypto_provider
            .as_ref()
            .map(|cp| &cp.signature_verification_algorithms)
            .unwrap_or(&default_provider.signature_verification_algorithms);
        verify_tls12_signature(message, cert, dss, algorithms)
    }

    fn verify_tls13_signature(
        &self,
        message: &[u8],
        cert: &CertificateDer<'_>,
        dss: &rustls::DigitallySignedStruct,
    ) -> Result<rustls::client::danger::HandshakeSignatureValid, rustls::Error> {
        let default_provider = rustls::crypto::ring::default_provider();
        let algorithms = self.crypto_provider
            .as_ref()
            .map(|cp| &cp.signature_verification_algorithms)
            .unwrap_or(&default_provider.signature_verification_algorithms);
        verify_tls13_signature(message, cert, dss, algorithms)
    }

    fn supported_verify_schemes(&self) -> Vec<rustls::SignatureScheme> {
        self.crypto_provider
            .as_ref()
            .map(|cp| cp.signature_verification_algorithms.supported_schemes())
            .unwrap_or_else(|| rustls::crypto::ring::default_provider().signature_verification_algorithms.supported_schemes())
    }
}

/// `ServerCertVerifier` that verifies that the server is signed by a trusted root, but allows any serverName
/// see the trait impl for more information.
#[derive(Debug)]
pub struct WebPkiVerifierAnyServerName {
    roots: RootCertStore,
    crypto_provider: Option<CryptoProvider>,
}

#[allow(unreachable_pub)]
impl WebPkiVerifierAnyServerName {
    pub fn new(roots: RootCertStore, crypto_provider: Option<CryptoProvider>) -> Self {
        Self { roots, crypto_provider }
    }
}