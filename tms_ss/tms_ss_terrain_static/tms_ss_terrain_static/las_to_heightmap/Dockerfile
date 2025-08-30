FROM debian:bullseye-20221219

ENV DEBIAN_FRONTEND = noninteractive

RUN apt-get update -y \
  && apt-get update -y

RUN apt-get install --no-install-recommends -y \
  build-essential \
  libpdal-dev \
  libpng-dev libpng++-dev

COPY . /build/

RUN cd /build && make

ENTRYPOINT ["/build/las2heightmap"]
