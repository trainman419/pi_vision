import orange
data = orange.ExampleTable("lenses")

print "Attribute"
for i in data.domain.attributes:
    print i.name
print 

print "Class:", data.domain.classVar.name

print "First five items:"
for i in range(5):
    print data[i]