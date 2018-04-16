all:
	nvcc BytomPoW.cu BytomPoW.cpp byte_order.c sha3.c  test_BytomPoW.cpp -I /opt/OpenBLAS/include/ -L/opt/OpenBLAS/lib -lopenblas -lpthread -I /usr/local/cuda/include -L /usr/local/cuda/lib64 -lcudart -lcublas

debug:
	nvcc -g BytomPoW.cu BytomPoW.cpp byte_order.c sha3.c  test_BytomPoW.cpp -I /opt/OpenBLAS/include/ -L/opt/OpenBLAS/lib -lopenblas -lpthread -I /usr/local/cuda/include -L /usr/local/cuda/lib64 -lcudart -lcublas
