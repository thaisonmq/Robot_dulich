#!/bin/sh
set -eu

certificate_directory=/etc/nginx/tls
ca_certificate_file="$certificate_directory/ca.crt"
ca_key_file="$certificate_directory/ca.key"
certificate_file="$certificate_directory/server.crt"
key_file="$certificate_directory/server.key"

if [ -s "$certificate_file" ] && [ -s "$key_file" ]; then
  exit 0
fi

tls_hostname=${TLS_HOSTNAME:-localhost}
mkdir -p "$certificate_directory"

case "$tls_hostname" in
  *:*) subject_alt_name="IP:$tls_hostname,DNS:localhost,IP:127.0.0.1" ;;
  *[!0-9.]*|'') subject_alt_name="DNS:$tls_hostname,DNS:localhost,IP:127.0.0.1" ;;
  *) subject_alt_name="IP:$tls_hostname,DNS:localhost,IP:127.0.0.1" ;;
esac

echo "Tạo CA nội bộ và chứng thư HTTPS cho $tls_hostname (chỉ dùng LAN/dev)."
openssl req -x509 -nodes -newkey rsa:3072 \
  -days "${TLS_CA_DAYS:-3650}" \
  -keyout "$ca_key_file" \
  -out "$ca_certificate_file" \
  -subj "/CN=ROVERA Local Development CA" \
  -addext "basicConstraints=critical,CA:TRUE,pathlen:0" \
  -addext "keyUsage=critical,keyCertSign,cRLSign" \
  >/dev/null 2>&1

temporary_directory=$(mktemp -d)
trap 'rm -rf "$temporary_directory"' EXIT HUP INT TERM
openssl req -nodes -newkey rsa:2048 \
  -keyout "$key_file" \
  -out "$temporary_directory/server.csr" \
  -subj "/CN=$tls_hostname" \
  >/dev/null 2>&1
printf '%s\n' \
  "basicConstraints=critical,CA:FALSE" \
  "keyUsage=critical,digitalSignature,keyEncipherment" \
  "extendedKeyUsage=serverAuth" \
  "subjectAltName=$subject_alt_name" \
  >"$temporary_directory/server.ext"
openssl x509 -req \
  -in "$temporary_directory/server.csr" \
  -CA "$ca_certificate_file" \
  -CAkey "$ca_key_file" \
  -set_serial 1 \
  -days "${TLS_CERT_DAYS:-825}" \
  -out "$certificate_file" \
  -extfile "$temporary_directory/server.ext" \
  >/dev/null 2>&1
chmod 600 "$ca_key_file"
chmod 644 "$ca_certificate_file"
chmod 600 "$key_file"
chmod 644 "$certificate_file"
