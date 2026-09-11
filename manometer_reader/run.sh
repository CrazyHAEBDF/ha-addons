#!/usr/bin/with-contenv bashio
set -e

bashio::log.info "Starte Manometer Reader ..."
exec python3 /gauge_reader.py
