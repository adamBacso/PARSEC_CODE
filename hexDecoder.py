while (True):
    hexData = input("Hex data: ")
    print(bytearray.fromhex(hexData).decode())