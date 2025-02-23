# import numpy as np
# import tensorflow as tf
#
# if __name__ == '__main__':
#     # Load the dataset
#     csv = np.loadtxt('data.csv', delimiter=',')
#     print(csv.shape)
#
#     # Separate features (x) and labels (y)
#     x = csv[:, 0:9]
#     y = csv[:, 9:11]
#
#     # Create a Linear Regression model
#     model = tf.keras.models.Sequential([
#         tf.keras.layers.Dense(2, input_shape=(9,), activation='linear')  # No hidden layers, linear activation
#     ])
#
#     # Compile the model with Mean Squared Error loss
#     model.compile(
#         optimizer=tf.keras.optimizers.SGD(learning_rate=0.01),  # Stochastic Gradient Descent
#         loss='mean_squared_error',
#         metrics=['mse']  # Evaluate using MSE metric
#     )
#
#     # Train the model
#     model.fit(x, y, epochs=200, verbose=1)
#
#     # Evaluate the model
#     loss = model.evaluate(x, y)
#     print(f"Final loss (MSE): {loss}")
#
#     # Save the model
#     model.save('linear_regression_model.keras')


import numpy as np
import tensorflow as tf
from tensorflow.python.keras.saving.saved_model.serialized_attributes import metrics

##LOAD THE DATA##
if __name__ == '__main__':
    csv = np.loadtxt('data.csv', delimiter=',')
    print(csv.shape)

    #creating the model
    model = tf.keras.models.Sequential([
        tf.keras.layers.Dense(9, input_shape=(9,), activation='tanh'), ## am observat rezultatele cu mai multe functii de activare
        tf.keras.layers.Dense(2),
    ])

    model.compile(
        # optimizer=tf.keras.optimizers.SGD(learning_rate=0.005, momentum=0.2),
        optimizer=tf.keras.optimizers.SGD(learning_rate=0.0025), ##folosim mai multi algoritmi si comparam rezultatele
        loss='mean_squared_error', ##afisam eroarea
        metrics=['accuracy', ''] ##afisam acuratetea
    )
    x = csv[:, 0:9] ##inputs
    y = csv[:, 9:11] ##outputs
    model.fit(x, y, epochs=300) ##de cate ori trece prin datele prezente in fisier

    loss = model.evaluate(x, y)
    print(loss) ##afisam datele de analiza

    model.save('model.keras') ##salvam modelul odata antrenat

