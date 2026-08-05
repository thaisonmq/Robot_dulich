# syntax=docker/dockerfile:1.7

FROM golang:1.26.0-bookworm AS gstreamer-publisher-builder

ARG GSTREAMER_PUBLISHER_COMMIT=3cc870ffb42931d730ff77326207e97555a50860
COPY gstreamer-publisher-duration.patch /tmp/gstreamer-publisher-duration.patch
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer1.0-dev \
        pkg-config \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src/gstreamer-publisher
RUN git clone https://github.com/livekit/gstreamer-publisher.git . \
    && git checkout "${GSTREAMER_PUBLISHER_COMMIT}" \
    && git apply /tmp/gstreamer-publisher-duration.patch \
    && gofmt -w main.go publish.go track.go
RUN GOMAXPROCS=2 CGO_ENABLED=1 go build -p=1 \
    -trimpath \
    -ldflags="-s -w" \
    -o /out/gstreamer-publisher .


FROM python:3.12.11-slim-bookworm AS base

ARG ROVERA_EDGE_VERSION=1.1.0
LABEL org.opencontainers.image.title="ROVERA Robot Edge"
LABEL org.opencontainers.image.version="${ROVERA_EDGE_VERSION}"

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    XDG_CACHE_HOME=/tmp/rovera-cache \
    XDG_CONFIG_HOME=/tmp/rovera-config

WORKDIR /app

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        alsa-utils \
        ca-certificates \
        ffmpeg \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-tools \
        pulseaudio-utils \
        tini \
        v4l-utils \
    && rm -rf /var/lib/apt/lists/*

COPY --from=gstreamer-publisher-builder \
    /out/gstreamer-publisher /usr/local/bin/gstreamer-publisher
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["python", "-m", "simulator.main"]


# RK3588 build: compile MPP only when the board is explicitly selected. ARM64
# alone is not sufficient: Raspberry Pi 5 is ARM64 but cannot use Rockchip MPP,
# and compiling that stack on a 4 GB Pi can exhaust swap during deployment.
FROM debian:bookworm-slim AS rk-media-builder

ARG MEDIA_PLATFORM=generic
ARG ROCKCHIP_MPP_COMMIT=df4864bd1e907cbfd427c397348976c5b2b05ee9
ARG FFMPEG_ROCKCHIP_COMMIT=388741a3544b92cf525f1cb3746ba9fb8f301d9a
RUN mkdir -p /opt/rockchip /opt/ffmpeg-rk \
    && if [ "$(dpkg --print-architecture)" = "arm64" ] \
        && [ "${MEDIA_PLATFORM}" = "rk3588" ]; then \
        apt-get update \
        && apt-get install -y --no-install-recommends \
            build-essential \
            ca-certificates \
            cmake \
            git \
            libasound2-dev \
            libdrm-dev \
            libpulse-dev \
            libssl-dev \
            nasm \
            pkg-config \
            yasm \
        && rm -rf /var/lib/apt/lists/*; \
    fi
RUN if [ "$(dpkg --print-architecture)" = "arm64" ] \
        && [ "${MEDIA_PLATFORM}" = "rk3588" ]; then \
        git clone https://github.com/rockchip-linux/mpp.git /src/mpp \
        && git -C /src/mpp checkout "${ROCKCHIP_MPP_COMMIT}" \
        && cmake \
            -S /src/mpp \
            -B /build/mpp \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_INSTALL_PREFIX=/opt/rockchip \
            -DBUILD_SHARED_LIBS=ON \
        && cmake --build /build/mpp --parallel 2 \
        && cmake --install /build/mpp; \
    fi
RUN if [ "$(dpkg --print-architecture)" = "arm64" ] \
        && [ "${MEDIA_PLATFORM}" = "rk3588" ]; then \
        git clone https://github.com/nyanmisaka/ffmpeg-rockchip.git /src/ffmpeg \
        && git -C /src/ffmpeg checkout "${FFMPEG_ROCKCHIP_COMMIT}" \
        && cd /src/ffmpeg \
        && PKG_CONFIG_PATH=/opt/rockchip/lib/pkgconfig:/opt/rockchip/lib/aarch64-linux-gnu/pkgconfig \
            ./configure \
                --prefix=/opt/ffmpeg-rk \
                --disable-debug \
                --disable-doc \
                --disable-static \
                --enable-gpl \
                --enable-libdrm \
                --enable-openssl \
                --enable-rkmpp \
                --enable-shared \
                --enable-version3 \
        && make -j2 \
        && make install; \
    fi


FROM base AS final
COPY --from=rk-media-builder /opt/rockchip /opt/rockchip
COPY --from=rk-media-builder /opt/ffmpeg-rk /opt/ffmpeg-rk
ENV LD_LIBRARY_PATH=/opt/ffmpeg-rk/lib:/opt/rockchip/lib:/opt/rockchip/lib/aarch64-linux-gnu

# Install VA-API userspace drivers only on x86_64. On ARM64 the image already
# contains the RK3588 MPP build above. This keeps one normal Compose command
# architecture-aware without requiring a manually selected Dockerfile.
RUN if [ "$(dpkg --print-architecture)" = "amd64" ]; then \
        apt-get update \
        && apt-get install -y --no-install-recommends \
            gstreamer1.0-vaapi \
            i965-va-driver \
            intel-media-va-driver \
            mesa-va-drivers \
            vainfo \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# Keep application source after the architecture-specific dependency layers so
# normal Python edits do not reinstall VA-API packages on every rebuild.
COPY simulator ./simulator
