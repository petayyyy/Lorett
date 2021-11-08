```bash
sudo apt install libnng-dev #(Y)
sudo apt install librtlsdr-dev libhackrf-dev libairspy-dev libairspyhf-dev  
sudo apt install libglew-dev libglfw3-dev #(Y)

git clone https://github.com/nanomsg/nng.git
cd nng
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr ..
make -j4
sudo make install
cd ../..
rm -rf nng

git clone https://github.com/altillimity/satdump.git
cd satdump
mkdir build && cd build
# If you want to build Live-processing (required for the ingestor), add -DBUILD_LIVE=ON to the command
# If you do not want to build the GUI Version, add -DNO_GUI=ON to the command
# If you want to disable some SDRs, you can add -DENABLE_SDR_AIRSPY=OFF or similar
cmake -DCMAKE_BUILD_TYPE=Release ..                             # MacOS
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr .. # Linux
make -j4
ln -s ../pipelines . # Symlink pipelines so it can run
ln -s ../resources . # Symlink pipelines so it can run
ln -s ../Ro* . # Symlink fonts for the GUI version so it can run

////////////////////////////////////////////////
wget http://www.fftw.org/fftw-3.3.9.tar.gz
tar xf fftw-3.3.9.tar.gz
rm fftw-3.3.9.tar.gz
cd fftw-3.3.9
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=false -DENABLE_FLOAT=true ..
make
sudo make install
cd ../..
rm -rf fftw-3.3.9

```
