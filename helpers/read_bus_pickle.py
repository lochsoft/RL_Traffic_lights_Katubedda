import pickle
import pandas as pd  # Optional, for DataFrame handling

# Path to your pickle file (update with your file's path)
pickle_file_path = "data/bus/bus_trips_morning_peak.pkl"

# Load the pickle file
try:
    with open(pickle_file_path, 'rb') as file:
        bus_trips = pickle.load(file)
    
    # Check the type of the loaded object
    print("Type of loaded object:", type(bus_trips))
    
    # Inspect the contents based on type
    if isinstance(bus_trips, pd.DataFrame):
        # If it's a DataFrame
        print("\nDataFrame Head:")
        print(bus_trips.head())
        print("\nDataFrame Columns:", bus_trips.columns.tolist())
        print("\nDataFrame Info:")
        print(bus_trips.info())
    elif isinstance(bus_trips, dict):
        # If it's a dictionary
        print("\nDictionary Keys:", bus_trips.keys())
        print("\nSample Values:")
        for key, value in list(bus_trips.items())[:5]:  # Show first 5 items
            print(f"{key}: {value}")
    elif isinstance(bus_trips, list):
        # If it's a list
        print("\nFirst 5 elements (if available):")
        print(bus_trips[:5])
    else:
        # For other types
        print("\nContents:", bus_trips)

except FileNotFoundError:
    print(f"Error: File {pickle_file_path} not found. Please check the path.")
except pickle.UnpicklingError:
    print("Error: Failed to unpickle the file. It may be corrupted or incompatible.")
except Exception as e:
    print(f"An error occurred: {str(e)}")