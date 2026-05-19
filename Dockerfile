FROM python:3.11-slim-bullseye

# Install C-compiler and utilities needed to download/extract source code
RUN apt-get update && apt-get install -y \
    build-essential \
    wget \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Download and compile the pigpio C-library from source to avoid OS-level package issues
WORKDIR /tmp
RUN wget https://github.com/joan2937/pigpio/archive/master.zip && \
    unzip master.zip && \
    cd pigpio-master && \
    make && \
    make install && \
    rm -rf /tmp/master.zip /tmp/pigpio-master

WORKDIR /opt/GalvoCon

# Install Python requirements
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application files
COPY . .

# Compile the native C extension inside the container during build
RUN gcc -shared -o galvo_core.so -fPIC galvo_core.c -lpigpio -lpthread

# Run the WebSocket server
EXPOSE 5000
CMD ["python", "app.py"]
