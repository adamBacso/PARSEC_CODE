sdBuffer = []

def base14_to_base10(number):
    return int(number, 14)

with open("Diagrams\\aszentlog.txt", "rt") as file:
    for line in file:
        printBuffer = ""
        line = line[len("radio_rx "):].strip()
        data = line.split("f")

        print(data)
        index = 0
        for dataFragment in data:
            try:
                if dataFragment != "":
                    if "e" in dataFragment:
                        dataFragmentParts = dataFragment.split("e")
                        printBuffer += str(base14_to_base10(dataFragmentParts[0]))
                        printBuffer += "."
                        printBuffer += str(base14_to_base10(dataFragmentParts[1]))
                    else:
                        if index == 1:
                            latitudeDecimal = base14_to_base10(dataFragment)
                            printBuffer += "47."
                            printBuffer += str(latitudeDecimal)
                        elif index == 2:
                            longitudeDecimal = base14_to_base10(dataFragment)
                            printBuffer += "19."
                            printBuffer += str(longitudeDecimal)
                        else:
                            printBuffer += str(base14_to_base10(dataFragment))
            except ValueError:
                print("ValueError: " + dataFragment)
                printBuffer += "~"

            printBuffer += ","
            index += 1

        print(printBuffer)
        sdBuffer.append(printBuffer)

with open("Diagrams\\aszentlogrendesen.csv", "w") as file:
    for line in sdBuffer:
        file.write(line)
        file.write("\n")