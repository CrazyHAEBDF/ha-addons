FROM ghcr.io/home-assistant/base:latest

ARG BUILD_VERSION
ARG BUILD_ARCH

LABEL \
  io.hass.version="${BUILD_VERSION}" \
  io.hass.type="app" \
  io.hass.arch="${BUILD_ARCH}"

RUN apk add --no-cache \
    python3 \
    py3-pip \
    py3-numpy \
    py3-opencv \
    ffmpeg \
  && pip3 install --no-cache-dir --break-system-packages "paho-mqtt==2.1.0"

COPY run.sh /run.sh
COPY instrument_reader.py /instrument_reader.py

RUN chmod a+x /run.sh

CMD ["/run.sh"]
