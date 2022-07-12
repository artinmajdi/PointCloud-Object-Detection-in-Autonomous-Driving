from matplotlib.ft2font import FIXED_SIZES
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import open3d
import pptk
from logging import raiseExceptions
import seaborn as sns

import tensorflow as tf
from tensorflow.keras import layers, models, optimizers, datasets, metrics
from collections import namedtuple


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

    '''
        'Range' = sqrt(x^2 + y^2 + z^2)
        'RCS'   =  20log( 'Amplitude' )
        'Index' is the rounded values for each feature. e.g.,
            - The 'Velocity' resolution is 0.16, hence the 'Velocity Index' = int(Velocity / 0.16).
            - The 'Azimuth' resolution is 1, hence the 'Azimuth Index' = int(Azimuth).
            - The 'Elevation' resolution is 1, hence the 'Elevation Index' = int(Elevation).
            - The 'Range' resolution is 0.5859, hence the 'Range Index' = int(Range / 0.5859). '''

    scale_min  = namedtuple("scale_min", ['Scale', 'Min'])

    SCALE_BIAS = {
                    'Range'     : scale_min(Scale=600, Min=0),
                    'Azimuth'   : scale_min(Scale=180, Min=0),
                    'Elevation' : scale_min(Scale=20 , Min=10),
                    'Velocity'  : scale_min(Scale=200, Min=100),
                    'RCS'       : scale_min(Scale=100, Min=0),
                    'Amplitude' : scale_min(Scale=1e5, Min=0),
                  }


    ORIGINAL_COLUMNS = {
        'Radar':['Frame ID', 'Pocket ID' , 'Y', 'X', 'Z', 'Range', 'Azimuth Angle', 'Elevation Angle', 'RCS' , 'Velocity' , 'Range Index', 'Azimuth Index', 'Elevation Index', 'Velocity Index', 'Amplitude Index', 'Timestamp', 'Temperature', '?'],
        'Lidar':['Y', 'X', 'Z', 'RCS' , 'Timestamp' , '?c'] }

    COLUMNS_RENAMING_MAPS = {'Range Index':'Range', 'Azimuth Index':'Azimuth', 'Elevation Index':'Elevation', 'RCS':'RCS'} # , 'Velocity Index':'Velocity'


    def __init__(self, modality = 'Radar', data_type='vector', normalize=True , dataset_directory='/home/artin/Documents/10dB_CR_10deg_in_2021-10-15-14-15-49', train_valid_test_ratio=[0.6, 0.2, 0.2]):

        self.dataset_directory = dataset_directory
        self.modality = modality
        self.data_type = data_type
        self.normalize = normalize
        self.train_valid_test_ratio = train_valid_test_ratio
        self.data = None


    @staticmethod
    def lidar_cartesian_to_spherical(dataframe: pd.DataFrame):

        ''' dataframe should contain ['X', 'Y', 'Z', 'RCS'] columns. '''

        for name in ['X','Y','Z','RCS']:
            assert name in dataframe.columns, f'{name} is not in the dataframe'


        # converting x,y,z to spherical angles/ranges
        Range, Elevation, Azimuth = cartesian_to_spherical(x=dataframe.X , y=dataframe.Y , z=dataframe.Z )

        # merging the spherical coordinates and the original dataframe
        df = dataframe.copy()

        # For details of the reason behind below calculations refer to the class docstring
        df['Azimuth Index']   = Azimuth.round()
        df['Elevation Index'] = Elevation.round().astype(int)
        df['Range Index']     = (Range / 0.5859).round().astype(int)
        df['Amplitude Index'] = np.power( df['RCS'] / 20.0 , 10 )

        return df


    @staticmethod
    def detect_and_remove_duplicated_points(df: pd.DataFrame):

        # First we sort the values based on RCS
        df.sort_values(by=['RCS'], inplace=True)

        # Then we remove all the duplicate rows (similar Azimuth and Elevation) except for the
        # last occurance (which has the highest RCS due to previous step sorting)
        df.drop_duplicates( subset=['Azimuth','Elevation'], keep='last', inplace=True)

        # Finaly sort the dataframe back to its original index order
        df.sort_index(inplace=True)

        return df


    def _read_dataframe(self, dir):
        self.dataframe_original = pd.read_csv(dir)
        self.dataframe_original.columns = self.ORIGINAL_COLUMNS[self.modality]
        self.dataframe = self.dataframe_original.copy()


    def get_dataframe(self, dir: str):

        self._read_dataframe(dir=dir)

        # Converting Lidar cartesian coordinates to spherical
        if self.modality == 'Lidar':
            self.dataframe = self.lidar_cartesian_to_spherical(dataframe=self.dataframe)

        # Removing unnecessary columns
        self.dataframe = self.dataframe[ self.COLUMNS_RENAMING_MAPS.keys() ]

        # Rename columns
        self.dataframe.rename(columns=self.COLUMNS_RENAMING_MAPS, inplace=True)

        # remove_duplicates
        self.dataframe = self.detect_and_remove_duplicated_points(df=self.dataframe.copy())

        # Normalize the data to > 0 according to SCALE_BIAS
        for name in set(self.SCALE_BIAS.keys())  &  set(self.dataframe.columns):
            self.dataframe[name] = self.dataframe[name] + self.SCALE_BIAS[name].Min


        # Removing the Lidar values that are outside pre-specified Radar range
        # if self.modality == 'Lidar':
        self._filtering_values_outside_specified_range()

        # Normalizing the dataframe
        # self._normalizing_each_column_to_0_1(scale_bias=self.SCALE_BIAS)

        # Splitting the data into train, valid and test.
        # # self.data = self.Data(data_raw=self.dataframe, train_valid_test_ratio=self.train_valid_test_ratio)

    def _filtering_values_outside_specified_range(self):

        for name in set(self.SCALE_BIAS.keys())  &  set(self.dataframe.columns):

            self.dataframe = self.dataframe.loc[self.dataframe[name] >= 0].astype(int)
            self.dataframe = self.dataframe.loc[self.dataframe[name] < self.SCALE_BIAS[name].Scale].astype(int)


    def _normalizing_each_column_to_0_1(self, scale_bias: dict):

        # Getting the names in scale_bias that exist in dataframe as well
        columns = set(scale_bias.keys())  &  set(self.dataframe.columns)

        for name in columns:

            self.dataframe[name] = ( self.dataframe[name] - scale_bias[name].Min ) / scale_bias[name].Scale


    def get_data(self, filename = '1_.txt'):

        # Getting the dataframe
        self.get_dataframe( dir=f'{self.dataset_directory}/{self.modality}/{filename}' )

        # # Converting the dataframe to a matrix
        if self.data_type == 'matrix':
            self.data_matrix = self.vector_to_matrix_convertion(matrix_value='RCS')


    def vector_to_matrix_convertion(self, matrix_value='RCS'):


        # Creating an empty array to store the data
        self.data_matrix = np.zeros( (self.SCALE_BIAS['Elevation'].Scale , self.SCALE_BIAS['Azimuth'].Scale) )

        for _, row in self.dataframe.iterrows():

            self.data_matrix[ int(row.Elevation) , int(row.Azimuth) ] = row[matrix_value] if isinstance(matrix_value,str) else matrix_value

        return self.data_matrix


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
    def visualize(points=None, spherical=True, method='open3d'):

        # Converting the spherical coordinates to cartesian
        if spherical:
            x,y,z = spherical_to_cartesian(Azimuth_Angle=points.Azimuth , Elevation_Angle=points.Elevation , Range=points.Range)
            points = np.array([x,y,z]).T
        else:
            x,y,z = points.values()

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


def cartesian_to_spherical(x, y, z):

    def calculate_elevation_angle():

        xy = np.sqrt( x**2 + y**2 )

        elevation_angle_defined = {'from Z-axis down': np.arctan2(xy, z),
                                    'from XY-plane up': np.arctan2(z, xy)}

        return elevation_angle_defined['from XY-plane up'] * 180 / 3.14


    # Range: sqrt(x^2 + y^2 + z^2)
    Range = np.sqrt( x**2 + y**2 + z**2 )

    # Elevation Angle (theta): arctan( sqrt(x^2 + y^2) / z )
    Elevation = calculate_elevation_angle()

    # Azimuth Angle (phi): arctan(y/x)
    Azimuth = np.arctan2(y, x) * 180 / 3.14

    return Range, Elevation, Azimuth


def spherical_to_cartesian(Azimuth_Angle, Elevation_Angle, Range):

    Azimuth_Angle = Azimuth_Angle * 3.14 / 180
    Elevation_Angle = Elevation_Angle * 3.14 / 180

    x = Range * np.cos(Azimuth_Angle) * np.cos(Elevation_Angle)
    y = Range * np.sin(Azimuth_Angle) * np.cos(Elevation_Angle)
    z = Range * np.sin(Elevation_Angle)

    return x,y,z

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
