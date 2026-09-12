#!/usr/bin/with-contenv bashio
set -e
bashio::log.info "Starte Heizungs Instrument Reader 1.0.0 ..."
exec nice -n 10 python3 /instrument_reader.py
