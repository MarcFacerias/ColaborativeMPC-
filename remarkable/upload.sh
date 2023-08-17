#! /bin/bash
bash download.sh
python3 edit_md.py
pandoc notes.md -o aux/new_notes.pdf
pdfunite aux/notes.pdf aux/new_notes.pdf aux/notes_final.pdf
rm aux/notes.pdf aux/new_notes.pdf
mv aux/notes_final.pdf aux/notes.pdf
rm notes.md
./rmapi rm /phd/notes_codi/notes
./rmapi put aux/notes.pdf /phd/notes_codi
rm -r aux