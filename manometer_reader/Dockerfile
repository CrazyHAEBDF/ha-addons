FROM ghcr.io/home-assistant/base:python3.12-alpine

RUN apk add --no-cache ffmpeg
COPY requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

COPY run.sh /
COPY gauge_reader.py /
RUN chmod a+x /run.sh

CMD [ "/run.sh" ]
