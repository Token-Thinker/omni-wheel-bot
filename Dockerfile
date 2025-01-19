FROM espressif/idf-rust:esp32_latest
USER root
WORKDIR /owb
COPY . /owb
RUN chown -R esp:esp /owb
USER esp

ENV PATH="/home/esp/.rustup/toolchains/esp/xtensa-esp-elf/esp-14.2.0_20240906/xtensa-esp-elf/bin:/home/esp/.cargo/bin:$PATH"

CMD ["/bin/bash"]