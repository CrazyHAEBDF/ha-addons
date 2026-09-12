#!/usr/bin/with-contenv bashio
set -e
bashio::log.info "Starte Manometer Reader 0.4.2 ..."
exec nice -n 10 python3 /gauge_reader.py
