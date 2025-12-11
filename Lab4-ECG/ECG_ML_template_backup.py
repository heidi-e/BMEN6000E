import pandas as pd
import numpy as np
import wfdb
import ast
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.layers import Conv1D, MaxPooling1D, Flatten, Dense
import matplotlib.pyplot as plt
from sklearn.metrics import multilabel_confusion_matrix
from sklearn.metrics import multilabel_confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

def load_raw_data(df, sampling_rate, path):
    if sampling_rate == 100:
        data = [wfdb.rdsamp(path+f) for f in df.filename_lr]
    else:
        data = [wfdb.rdsamp(path+f) for f in df.filename_hr]
    data = np.array([signal for signal, meta in data])
    return data

# You will need to change this to wherever you downloaded the data to
path = '/Users/Heidi/Downloads/ptb-xl-a-large-publicly-available-electrocardiography-dataset-1.0.3/'
sampling_rate=100

#n_classes = 5
classes = ['NORM', 'MI', 'CD', 'STTC', 'HYP']

# load and convert annotation data
Y = pd.read_csv(path+'ptbxl_database.csv', index_col='ecg_id')
Y.scp_codes = Y.scp_codes.apply(lambda x: ast.literal_eval(x))

# Load raw signal data
X = load_raw_data(Y, sampling_rate, path)

# Load scp_statements.csv for diagnostic aggregation
agg_df = pd.read_csv(path+'scp_statements.csv', index_col=0)
agg_df = agg_df[agg_df.diagnostic == 1]

def aggregate_diagnostic(y_dic):
    tmp = []
    for key in y_dic.keys():
        if key in agg_df.index:
            tmp.append(agg_df.loc[key].diagnostic_class)
    return list(set(tmp))

# Apply diagnostic superclass
Y['diagnostic_superclass'] = Y.scp_codes.apply(aggregate_diagnostic)

# Code diagnoses as an array of binary values
Z = pd.DataFrame(0, index = Y.index, columns = classes, dtype = 'int')
for i in Z.index:
    for k in Y.loc[i].diagnostic_superclass:
        Z.loc[i, k] = 1

# Split data into train and test

# strat_fold is a random number 1-10 assigned by the creators of the dataset specifically for the purpose of splitting data into test and train datasets.
# this is what divides 90% training, 10% test
#test_fold_max = 9

# Include metadata of your choice; comment if not using
'''meta_data = np.array([Y.STTC]) # fill in your choice of metadata here; you can copy/paste more copies below to add more variables
meta_data = np.repeat(meta_data, 1000, axis = 0)
meta_data = meta_data.T
X = np.dstack((X,meta_data))'''

# Training data
#X_train = X[np.where(Y.strat_fold <= test_fold_max)]
#y_train = Z[(Y.strat_fold <= test_fold_max)]
# Test data
#X_test = X[np.where(Y.strat_fold > test_fold_max)]
#y_test = Z[Y.strat_fold > test_fold_max]

split_conditions = [1, 5, 9]  # corresponds to 10%, 20%, ..., 90%
results = []

for fold_max in split_conditions:
    print(f"\n🔹 Training on folds 1–{fold_max} | Testing on {fold_max+1}–10")

    X_train = X[np.where(Y.strat_fold <= fold_max)]
    y_train = Z[Y.strat_fold <= fold_max]
    X_test = X[np.where(Y.strat_fold > fold_max)]
    y_test = Z[Y.strat_fold > fold_max]

    # Create tensor dataset from raw data
    dataset = tf.data.Dataset.from_tensor_slices((X_train, y_train)).batch(1)
    eval_dataset = tf.data.Dataset.from_tensor_slices((X_test, y_test)).batch(1)

    model.fit(dataset, epochs=1, verbose=0)
    score = model.evaluate(eval_dataset, return_dict=True)
    results.append({'train_pct': fold_max * 10, **score})


# Create tensor dataset from raw data
#dataset = tf.data.Dataset.from_tensor_slices((X_train,y_train)).batch(1)
#eval_dataset = tf.data.Dataset.from_tensor_slices((X_test, y_test)).batch(1)

# Create a model with three layers: 1D CNN; 1D CNN; Dense
model = keras.Sequential([
    Conv1D(32, 5, activation='relu'),
    MaxPooling1D(2),   
    Conv1D(64, 3, activation='relu'),
    MaxPooling1D(2),
    # Data needs to be "flattened" before running through a dense NN
    Flatten(),               
    # Final layer needs to match the shape of the output. 
    # Sigmoid activation is for binary (Y/N) outputs
    Dense(Z.shape[-1], activation="sigmoid")               
])

# Tell the model what to optimize for
model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['binary_accuracy', 'Precision', 'Recall'])

# Run
model.fit(dataset, epochs=1)

# Report out metrics from model.compile; binary_accuracy is raw overall accuracy
accuracy_score = model.evaluate(eval_dataset, return_dict=True)

# Make predictions for the test data
predictions = model.predict(X_test)

# Find the locations of maximum certainty
max_locations = np.argmax(predictions, axis=1)
predicted_classes_max = np.zeros(predictions.shape).astype(int)
length = predictions.shape[0]
for i in range(length):
    predicted_classes_max[i,max_locations[i]] = 1

# Also find locations where certainty > threshold (to account for multiple diagnoses)
# You can change this threshold if you like
predicted_classes_thresh = (predictions > 0.6).astype(int)

# Union of "best" and "all other good" prediction matrices
all_predictions = predicted_classes_max | predicted_classes_thresh

# Confusion matrix
mlcm = multilabel_confusion_matrix(y_test, all_predictions)


labels = y_test.columns

fig, axes = plt.subplots(1, len(labels), figsize=(4 * len(labels), 4))

for i, label in enumerate(labels):
    cm = mlcm[i]
    ax = axes[i] if len(labels) > 1 else axes
    sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', ax=ax, cbar=False)
    ax.set_title(label)
    ax.set_xlabel('Predicted')
    ax.set_ylabel('Actual')
    ax.set_xticklabels(['Negative', 'Positive'])
    ax.set_yticklabels(['Negative', 'Positive'])

plt.tight_layout()
plt.show()




