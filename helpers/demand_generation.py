import generate_edge_relation as gen
import pandas as pd
import os

## setting working directory
os.chdir(r"C:\Users\Lochana Minuwansala\Downloads\Simulation  model\Katubedda Junction\helpers")

## Read traffic counts and generate edge relation file
counts_6am_7am = pd.read_excel("../data/traffic_counts_all.xlsx", sheet_name="katubedda_junction_6am_7am")
counts_warmup = pd.read_excel("../data/traffic_counts_all.xlsx", sheet_name="kbj_545am_6am_minor_warmup")

gen.generate_edge_relation_xml(count_df=counts_6am_7am, output_file='../data/edge_relation_data_6am_7am.dat.xml')
gen.generate_edge_relation_xml(count_df=counts_warmup, output_file='../data/edge_relation_data_warmup.dat.xml')