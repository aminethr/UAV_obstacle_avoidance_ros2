import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import xgboost as xgb
import joblib

# 1. Load the  dataset
data = pd.read_csv("dataset.csv")

# 2. Separate features and target
X = data[["Speed (m/s)"]]
y = data["Stopping Distance (m)"]

# 3. Split into train and test sets (80% train, 20% test)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# 4. Create the XGBoost Regressor
model = xgb.XGBRegressor(
    n_estimators=100,
    learning_rate=0.1,
    max_depth=5,
    subsample=0.8,
    colsample_bytree=0.8,
    random_state=42
)

# 5. Train the model
model.fit(X_train, y_train)

# 6. Make predictions
y_pred = model.predict(X_test)

# 7. Evaluation Metrics
rmse = np.sqrt(mean_squared_error(y_test, y_pred))
mae = mean_absolute_error(y_test, y_pred)
r2 = r2_score(y_test, y_pred)

print("===== MODEL EVALUATION =====")
print(f"R² Score: {r2:.4f}")
print(f"Mean Absolute Error (MAE): {mae:.4f} meters")
print(f"Root Mean Squared Error (RMSE): {rmse:.4f} meters")

# 8. Save the trained model to a file
joblib.dump(model, "xgboost_stopping_distance_model.joblib")
print("✅ Model saved successfully as 'xgboost_stopping_distance_model.joblib'")

