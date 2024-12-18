import numpy as np
import tensorflow as tf
from tensorflow.python.keras.saving.saved_model.serialized_attributes import metrics

if __name__ == '__main__':
    csv = np.loadtxt('data.csv', delimiter=',')
    print(csv.shape)

    # Create a model
    model = tf.keras.models.Sequential([
        tf.keras.layers.Dense(9, input_shape=(9,), activation='tanh'),
        tf.keras.layers.Dense(2),
    ])

    model.compile(
        # optimizer=tf.keras.optimizers.SGD(learning_rate=0.005, momentum=0.2),
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.0025),
        loss='mean_squared_error',
        metrics=['accuracy']
    )
    x = csv[:, 0:9]
    y = csv[:, 9:11]
    model.fit(x, y, epochs=200)

    loss = model.evaluate(x, y)
    print(loss)

    model.save('model.keras')
