import gdb


class VecfPrinter(gdb.ValuePrinter):
    def __init__(self, val):
        self.val = val

    def to_string(self):
        return "({x:.2f}, {y:.2f}, {z:.2f})".format(x=float(self.val["x"]), y=float(self.val["y"]), z=float(self.val["z"]))

    def children(self):
        for field in self.val.type.fields():
            yield field.name, field


def pp(val):
    if str(val.type) == "Vecf": return VecfPrinter(val)
    elif str(val.type) == "Vector": return VecfPrinter(val)
    elif str(val.type) == "Color": return VecfPrinter(val)
    else: return None

gdb.pretty_printers.append(pp)


