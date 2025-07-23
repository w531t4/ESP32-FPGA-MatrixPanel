1. Information can be found here `https://gitdemo.readthedocs.io/en/latest/linux-setup.html`
1. `wget https://dl.espressif.com/dl/xtensa-esp32-elf-linux64-1.22.0-61-gab8375a-5.2.0.tar.gz`
1. get esp-idf, place in toolchain `git clone --recursive https://github.com/espressif/esp-idf.git`
1. `cd toolchain/esp-idf; git submodule update --init`
1. get template idf project `git clone https://github.com/espressif/esp-idf-template.git myapp`
1. move contents of myapp into primary directory.
1. in esp-idf dir, execute `./install.sh`
1. load env (From export.sh) when seeking to interface with library
