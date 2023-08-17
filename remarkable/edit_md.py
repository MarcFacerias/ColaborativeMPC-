from datetime import date

today = date.today()
d2 = today.strftime("%B %d, %Y")

# Load the file into file_content
file_content = [ line for line in open('../notes.md') ]

# Overwrite it
writer = open('notes.md','w')
writer.write(d2 + "\n"+ "\n")

for line in file_content:
    writer.write(line)


writer.close()

# Insert text at the beginning of the document.


