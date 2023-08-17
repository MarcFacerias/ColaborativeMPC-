#! /bin/bash
./rmapi get /phd/notes_codi/notes
unzip "notes.zip" *.pdf -d aux
mv aux/*.pdf aux/notes.pdf
cp aux/notes.pdf ../notes.pdf
rm notes.zip

