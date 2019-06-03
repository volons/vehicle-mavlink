FROM python:2.7-slim-jessie

RUN apt-get update && \
	apt-get install -y build-essential && \
	rm -rf /var/lib/apt/lists/*

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY ./src /app

WORKDIR /app
ENV PYTHONUNBUFFERED=0
CMD python ./main.py
