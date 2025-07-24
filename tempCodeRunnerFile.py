temp = pd.read_csv(f"outputs/dynamic_vehicle_data/processed_dynamic_vehicle_data.csv")
temp['scenario'] = "dynamic"

all_pd.to_csv(f'outputs/dynamic_vehicle_data/processed_dynamic_vehicle_data.csv', index = False)