# obey

C++ SocketCAN OBD-II tool

## Functions
- Request (read) Servcice/PID
- Scan/clear fault codes
- Enumerate ECUs

## Building
```sh
mkdir build
cd build
cmake ..
make
```

## Testing

obdsim is a server that pretends to be a car ECU to test the tool

```sh
sudo modprobe vcan
sudo ip link add vcan0 type vcan
sudo ip link set up vcan0

./obdsim

```

Then run obey tool with args `-i vcan0`
