FROM ghcr.io/home-assistant/base:python3.12

RUN apt-get update \
  && apt-get install -y --no-install-recommends ffmpeg \
  && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

COPY run.sh /
COPY gauge_reader.py /
RUN chmod a+x /run.sh

CMD [ "/run.sh" ]
