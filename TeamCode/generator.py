#!/usr/bin/env python3
import os
import shlex

class ServoPositionProcessor:
    def __init__(self):
        self.positions = {}

    def process(self, fname, line, name, params):
        if len(params) < 2:
            print("ERROR: Two parameters required: <servo> <name>")
            return
        servo = params[0]
        name = params[1]
        if not servo in self.positions:
            self.positions[servo] = []
        self.positions[servo].append(name)

    def finish(self):
        with open('preset_names.txt', 'w') as f:
            for servo in self.positions.keys():

                f.write(servo + ':')
                i = 0
                for pos in self.positions[servo]:
                    f.write(pos)
                    if i < len(self.positions[servo]) - 1:
                        f.write(',')
                    i += 1
                f.write('\n')


class Processor:
    def process(self, fname, line, name, params):
        pass

    def finish(self):
        pass

def checkFile(file, processors):
    with open(file, 'r') as f:
        lineno = 0
        for line in f:
            lineno += 1   # File lines appear to start at 1
            line = line.strip()
            if line.startswith("//#"):
                split = shlex.split(line)
                name = split[0][3:]
                params = split[1:]
                if name not in processors.keys():
                    print("WARNING: Processor not found for %s (line=%d in %s)" % (name, lineno, file))
                    continue

                processors[name].process(file, lineno, name, params)

def main():
    print("Running custom file processors")
    # Register processors here
    processors = {
        "position": ServoPositionProcessor()
    }


    for root, dirs, files in os.walk('src/main/java/'):
        for file in files:
            if file.endswith('.java'):
                checkFile(root + '/' + file, processors)

    for processor in processors.values():
        processor.finish()

if __name__ == "__main__":
    main()