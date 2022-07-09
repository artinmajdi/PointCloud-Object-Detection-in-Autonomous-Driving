import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import open3d
import pptk
from logging import raiseExceptions
import seaborn as sns


import tensorflow as tf
from tensorflow.keras import layers, models, optimizers, datasets, metrics



class Optimization():

    def __init__(self):
        self.model = None

    def fit(self, epochs=5, batch_size=32, train=None, valid=None):

        self.model = self.architecture(input_shape=(train.shape[1],1))

        self.model.fit(x=train, y=train, epochs=epochs, batch_size=batch_size, validation_data=(valid, valid))

    def architecture(self, input_shape: tuple):

        input = layers.Input(shape=input_shape)

        # Encoder
        x = layers.Conv1D(filters=256, kernel_size=3, strides=1, padding='same', activation='relu')(input)
        x = layers.Conv1D(filters=512, kernel_size=3, strides=1, padding='same', activation='relu')(x)
        x = layers.Conv1D(filters=512, kernel_size=3, strides=2, padding='same', activation='relu')(x)

        # Decoder
        x = layers.Conv1DTranspose(filters=256, strides=2, kernel_size=3, padding='same', activation='relu')(x)
        x = layers.Conv1DTranspose(filters=256, strides=1, kernel_size=3, padding='same', activation='relu')(x)
        x = layers.Conv1DTranspose(filters=1,   strides=1, kernel_size=3, padding='same', activation='sigmoid')(x)


        # Autoencoder
        model = models.Model(input, x)
        model.compile(optimizer="adam", loss="binary_crossentropy")

        return model

    def predict(self, test):

        predictions = self.model.predict(test)

        if isinstance(test, pd.DataFrame):
            return pd.DataFrame(predictions[:,:,0], columns=test.columns, index=test.index)
        return predictions


class DataLoader():

    def __init__(self, modality = 'Radar', data_type='vector', normalize=True , dataset_directory='/home/artin/Documents/10dB_CR_10deg_in_2021-10-15-14-15-49', train_valid_test_ratio=[0.6, 0.2, 0.2]):

        self.dataset_directory = dataset_directory
        self.modality = modality
        self.data_type = data_type
        self.normalize = normalize
        self.train_valid_test_ratio = train_valid_test_ratio
        self.data = None

        if modality == 'Radar':
            self.columns = ['Frame ID', 'Pocket ID' , 'Y', '-X', 'Z', 'Range', 'Azimuth Angle', 'Elevation Angle', 'RCS' , 'Velocity' , 'Range Index', 'Azimuth Index', 'Elevation Index', 'Velocity Index', 'Amplitude Index', 'Timestamp', 'Temperature', '?']
            self.rename_columns_dict = {'Range Index':'Range', 'Azimuth Index':'Azimuth', 'Elevation Index':'Elevation', 'Amplitude Index':'Amplitude', 'RCS':'RCS', 'Velocity Index':'Velocity'}

        elif modality == 'Lidar':
            self.columns = ['X', 'Y', 'Z', '?a' , '?b' , '?c']
            self.rename_columns_dict =  {'X':'X', 'Y':'Y', 'Z':'Z', '?a':'?a', '?b':'?b' , '?c':'?c'}

    def get_dataframe(self, dir: str):

        def _read_data(dir):
            self.dataframe_original = pd.read_csv(dir)
            self.dataframe_original.columns = self.columns
            self.dataframe = self.dataframe_original.copy()

        def _filter_columns(columns: list):
            self.dataframe = self.dataframe[columns]

        def _rename_columns(columns: dict):
            self.dataframe.rename(columns=columns, inplace=True)

        _read_data(dir=dir)
        _filter_columns( columns=self.rename_columns_dict.keys() )
        _rename_columns( columns=self.rename_columns_dict )

    def get_data(self, filename = '1_.txt'):
        ''' data_type can be 'vector' or 'matrix' '''

        dir = f'{self.dataset_directory}/{self.modality}/{filename}'

        self.get_dataframe(dir=dir)

        data_raw = self.postprocess_dataframe(modality=self.modality, normalize=self.normalize)

        # Puting the data into a class and splitting the data into train, valid and test.
        self.data = self.Data(data_raw=data_raw, train_valid_test_ratio=self.train_valid_test_ratio)

        return self.data

    def postprocess_dataframe(self, modality='Radar', normalize=True):

        def conversion_to_3D( matrix_dimensions={'Range':600, 'Azimuth':180, 'Elevation':20}, matrix_value='Amplitude'):

            ''' Range is assumed to be in the range [0, 600]
                Elevation Angle is in the range [-10, 10]
                Azimuth angle is in the range [0, 180]
                matrix_value can either be an integer/float or a string that corresponds to a column name. It's value will be used to fill the matrix. '''

            # Creating an empty array to store the data
            self.data_matrix = np.zeros( list(matrix_dimensions.values()) )

            # Shifting the Elevation values to the range (0, 20)
            self.dataframe.Elevation += 10


            # dimension = len(self.data_matrix.shape)

            for _, row in self.dataframe.iterrows():

                self.data_matrix[ tuple( row[matrix_dimensions.keys()].astype(int).to_list() ) ] = row[matrix_value] if isinstance(matrix_value,str) else matrix_value

                # if dimension == 3:
                #     self.data_matrix [ int(row[x]) ] [ int(row[y]) ] [ int(row[z]) ] = row[matrix_value] if isinstance(matrix_value,str) else matrix_value
                #     x,y,z = matrix_dimensions.keys()

                # elif dimension == 2:
                #     self.data_matrix [ int(row[x]) ] [ int(row[y]) ] = row[matrix_value] if isinstance(matrix_value,str) else matrix_value
                #     x,y = matrix_dimensions.keys()


            return self.data_matrix

        def normalizing_to_0_1( normalization_values = {'Range':1000, 'Azimuth':180, 'Elevation':20, 'RCS':100, 'Velocity':200, 'Amplitude':100000} ):


            for key, value in normalization_values.items():
                self.dataframe[key] /= value

            self.dataframe.Elevation += 0.5
            self.dataframe.Velocity  += 0.5

            return self.dataframe


        if modality == 'Radar':

            if self.data_type == 'vector':  return normalizing_to_0_1() if self.normalize else self.dataframe

            elif self.data_type == 'matrix': return conversion_to_3D( matrix_dimensions={'Azimuth':180, 'Elevation':20}, matrix_value='Range')

            else: raise ValueError('data_type can be either "vector" or "matrix"')

        elif modality == 'Lidar':
            return self.dataframe

    class Data:
        def __init__(self, data_raw=None , train_valid_test_ratio: list=[3,1,1]):
            self.full  = data_raw
            self.train = None
            self.valid = None
            self.test  = None

            self._separate_train_valid_test_only_for_dataframe(train_valid_test_ratio=train_valid_test_ratio)

        def _separate_train_valid_test_only_for_dataframe(self, train_valid_test_ratio=None):

            if train_valid_test_ratio is None:
                return

            data = self.full.copy()

            frac = {}
            for ix, mode in enumerate(['train' , 'valid' , 'test']):
                frac[mode] = train_valid_test_ratio[ix]/sum(train_valid_test_ratio)


            self.train = data.sample(frac=frac['train'], random_state=42)
            data.drop(self.train.index)

            self.valid = data.sample(frac=frac['valid'], random_state=42)
            data.drop(self.valid.index)

            self.test  = data.copy()

        @property
        def shape(self):
            return self.full.shape

    @staticmethod
    def appendSpherical_np(xyz):

        ptsnew = np.hstack((xyz, np.zeros(xyz.shape)))

        xy = xyz[:,0]**2 + xyz[:,1]**2

        ptsnew[:,3] = np.sqrt(xy + xyz[:,2]**2)

        # for elevation angle defined from Z-axis down
        # ptsnew[:,4] = np.arctan2(np.sqrt(xy), xyz[:,2])

        # for elevation angle defined from XY-plane up
        ptsnew[:,4] = np.arctan2(xyz[:,2], np.sqrt(xy))

        ptsnew[:,5] = np.arctan2(xyz[:,1], xyz[:,0])

        return ptsnew

    @classmethod
    def visualize(cls, points=None, method='open3d', run_demo=False, modality='Lidar', filename='1_.txt'):
        '''
        Example: Run from point clouds:
            >> data = DataLoader(modality='Lidar').get_data(filename='1_.txt')
            >> DataLoader().visualize( points=data.train[data.train.columns[:3]].to_numpy()  , method='open3d' )

        Example: Run from demo sample:
            >> DataLoader().visualize(run_example=True, modality='Lidar', filename='2_.txt')
        '''

        if run_demo and points is None:
            data   = cls(modality=modality).get_data(filename=filename)
            points = data.train[data.train.columns[:3]].to_numpy()


        if method == 'open3d':

            pcd = open3d.geometry.PointCloud()
            pcd.points = open3d.utility.Vector3dVector(points)
            # pcd.colors = open3d.utility.Vector3dVector(data[data.columns[3]].to_numpy())
            open3d.visualization.draw_geometries([pcd])


        elif method == 'pptk':

            # points = pptk.points(points)
            v = pptk.viewer(points)
            # v.attribute('color', data[data.columns[3]].to_numpy())

        else:
            raiseExceptions('method should be either "open3d" or "pptk"')


class VectorInput(DataLoader, Optimization):

    def __init__(self, dataset_directory='/home/artin/Documents/10dB_CR_10deg_in_2021-10-15-14-15-49' , modality='Radar', normalize=True, filename='1_.txt', train_valid_test_ratio=[3,1,1]):

        # Loading the data
        DataLoader.__init__( self, data_type='vector' , dataset_directory=dataset_directory, modality=modality, normalize=normalize, train_valid_test_ratio=train_valid_test_ratio)
        self.get_data(filename=filename)

        # Getting the architecture of the model
        Optimization.__init__(self)

        # Training the model
        self.fit(epochs=5, batch_size=32, train=self.data.train, valid=self.data.valid)

        # Testing the model
        self.prediction = self.predict(test=self.data.test)



class MatrixInput(DataLoader, Optimization):

    def __init__(self, dataset_directory='/home/artin/Documents/10dB_CR_10deg_in_2021-10-15-14-15-49' , modality='Radar', train_valid_test_ratio=None, normalize=True, filename='1_.txt'):

        # Loading the data
        DataLoader.__init__(self, data_type='matrix' , modality=modality, dataset_directory=dataset_directory, normalize=True, train_valid_test_ratio=train_valid_test_ratio)
        self.get_data(filename=filename)

        # Getting the architecture of the model
        # Optimization.__init__(self)

        # Training the model
        # self.fit(epochs=5, batch_size=32, train=self.data.train, valid=self.data.valid)

        # Testing the model
        # self.predictions = self.predict(test=self.data.test)




    def view(self):

        test = self.data.test.reset_index()
        predictions = self.prediction.reset_index()

        sns.set()
        for y in test.columns[1:]:

            plt.figure(figsize=(12, 6))
            sns.scatterplot(x='index', y=y, data=test, label='data.test')
            sns.scatterplot(x='index', y=y, data=predictions, label='predictions')
            plt.legend()
            plt.xlabel('index')
            plt.show()
