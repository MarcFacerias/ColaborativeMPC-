import pandas as pd
import os

# USED TO FIX SOME ISSUES WITH THE NL TIME TRACK SO THAT IT WORKS WITH MATLAB

files_CS = "data/experiments_paper/NL_3agents_def/csv/"
files = os.listdir(files_CS)
df = [''] * len(files)
for i,path in enumerate(files):
    df[i] = pd.read_csv(files_CS + path +'/time.dat', sep=" ", header=None).astype(float)
    df[i].to_csv(files_CS + path +"/time_ocd.dat", sep=" ", header=None, index = False)
    fx = df[i].sum(axis=1)
    fx.to_csv(files_CS + path +"/time_def.dat", sep=" ", header=None, index = False)