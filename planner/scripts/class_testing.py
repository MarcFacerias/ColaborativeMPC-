class aux():
    def __init__(self):
        self.inner = "test"

class Person:
    def __init__(self, name, last_name, age):

        self.last_name = last_name
        self.age = age
        self.ic = aux()
        self.name = self.ic.inner

    def set_name(self,name):
        self.ic.inner = name

class Student(Person):
    def __init__(self, indexNr, notes, **kwargs):
        # since Employee comes after Student in the mro, pass its arguments using super
        super().__init__(**kwargs)
        self.indexNr = indexNr
        self.notes = notes

class Employee(Person):
    def __init__(self, salary, position, **kwargs):
        super().__init__(**kwargs)
        self.salary = salary
        self.position = position

class WorkingStudent(Student, Employee):
    def __init__(self, **kwargs):
        # pass all arguments along the mro
        super().__init__(**kwargs)

# keyword arguments (not positional arguments like the case above)
ws = WorkingStudent(name="john", last_name="brown", age=18, indexNr=1, notes=[1,2,3], salary=1000, position='Programmer')
ws2 = WorkingStudent(name="pepe", last_name="tet", age=20, indexNr=1, notes=[4,5,6], salary=1000, position='yrdtset')
print(ws2.name)
print(ws.name)
ws.set_name("pepe")
print(ws2.ic.inner)
print(ws.ic.inner)
