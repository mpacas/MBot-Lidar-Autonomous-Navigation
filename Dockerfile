FROM gcc:8.4

WORKDIR /

# Install dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        libssl-dev \
        libboost-system-dev \
        libboost-thread-dev \
        libboost-coroutine-dev \
        libboost-context-dev

# Webapp dependencies.
RUN curl -fsSL https://deb.nodesource.com/setup_14.x | bash
RUN apt-get install -y nodejs

# Get webapp code.
ADD https://api.github.com/repos/rob102-staff/nav-app/git/refs/heads/main version.json
RUN git clone https://github.com/rob102-staff/nav-app.git /app

# Copy over the necessary packages. The main development directory will be mounted.
COPY data/ /code/data/
COPY CMakeLists.txt /code/

WORKDIR /app
RUN npm install

CMD ["npm", "start"]
