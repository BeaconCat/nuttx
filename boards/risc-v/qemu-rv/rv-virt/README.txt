1. Download and install toolchain

  $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz

2. Build and install qemu

  $ git clone https://github.com/qemu/qemu
  $ cd qemu
  $ ./configure --target-list=riscv32-softmmu,riscv64-softmmu
  $ make
  $ sudo make install

3. Configure and build NuttX

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://github.com/apache/incubator-nuttx.git nuttx
  $ git clone https://github.com/apache/incubator-nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh rv-virt:nsh
  $ make

4. Run the nuttx with qemu

  $ qemu-system-riscv32 -semihosting -M virt -cpu rv32 -smp 8 -bios none -kernel nuttx -nographic

  or

  $ qemu-system-riscv64 -semihosting -M virt -cpu rv64 -smp 8 -bios none -kernel nuttx -nographic

  NuttShell (NSH) NuttX-10.3.0-RC1
  nsh> mount -t hostfs -o fs=. /host
  nsh> cat /host/AUTHORS
  This is a list of all the contributors that have submitted ICLA, SGA
  If you are not on this list and believe you should be, please inform us.

  ICLA
  ====
  ...
  nsh>

4. Run the nuttx network with qemu

  $ qemu-system-riscv32 -semihosting -M virt,aclint=on -cpu rv32 -smp 8 \
  -global virtio-mmio.force-legacy=false \
  -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
  -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.0 \
  -bios none -kernel nuttx -nographic

  or

  $ qemu-system-riscv64 -semihosting -M virt,aclint=on -cpu rv64 -smp 8 \
  -global virtio-mmio.force-legacy=false \
  -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
  -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.0 \
  -bios none -kernel nuttx -nographic

5. TODO

  Support FPU
  Support RISC-V User mode
