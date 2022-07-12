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


    SCALE_BIAS = {
        'Range':{'Scale':600 , 'Min':0},
        'Azimuth':{'Scale':180 , 'Min':0},
        'Elevation':{'Scale':20 , 'Min':10},
        'Velocity':{'Scale':200 , 'Min':100},
        'RCS':{'Scale':100 , 'Min':0},
        'Amplitude':{'Scale':1e5 , 'Min':0},
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
        df, _ = cartesian_to_spherical(xyz = dataframe[['X','Y','Z']].to_numpy())

        # merging the spherical coordinates and the original dataframe
        df = pd.concat([dataframe[['Timestamp' , 'RCS']], df], axis=1)

        # For details of the reason behind below calculations refer to the class docstring
        df['Azimuth Index']   = df['Azimuth Angle'].round()
        df['Elevation Index'] = df['Elevation Angle'].round().astype(int)
        df['Range Index']     = (df['Range'] / 0.5859).round().astype(int)
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


    def get_dataframe(self, dir: str):

        def _read_data(dir):
            self.dataframe_original = pd.read_csv(dir)
            self.dataframe_original.columns = self.ORIGINAL_COLUMNS[self.modality]
            self.dataframe = self.dataframe_original.copy()

        _read_data(dir=dir)

        # Converting Lidar cartesian coordinates to spherical
        if self.modality == 'Lidar':
            self.dataframe = self.lidar_cartesian_to_spherical(dataframe=self.dataframe)

        # Fitler columns
        self.dataframe = self.dataframe[ self.COLUMNS_RENAMING_MAPS.keys() ]

        # Rename columns
        self.dataframe.rename(columns=self.COLUMNS_RENAMING_MAPS, inplace=True)

        # remove_duplicates
        self.dataframe2 = self.detect_and_remove_duplicated_points(df=self.dataframe.copy())


        # # Normalizing the dataframe
        # self._normalizing_each_column_to_0_1(scale_bias=self.SCALE_BIAS)

        # Splitting the data into train, valid and test.
        # # self.data = self.Data(data_raw=self.dataframe, train_valid_test_ratio=self.train_valid_test_ratio)


    def _normalizing_each_column_to_0_1(self, scale_bias={'feature1':{'Scale':1,'Min':0}}):

        for name in scale_bias:
            if name in self.dataframe.columns:
                A, mn = scale_bias[name]['Scale'], scale_bias[name]['Min']
                self.dataframe[name] = (self.dataframe[name] - mn) / A



    def get_data(self, filename = '1_.txt'):

        # Getting the dataframe
        self.get_dataframe( dir=f'{self.dataset_directory}/{self.modality}/{filename}' )

        # # Converting the dataframe to a matrix
        # if self.data_type == 'matrix':
        #     self.data_matrix = self.vector_to_matrix_convertion()


        # return self.data



    def vector_to_matrix_convertion(self):
        ''' source_dataframe_columns Examples:
                'Cartesian': ['X', 'Y', 'Z']
                'Spherical': ['Range','Azimuth','Elevation']
        '''

        # Creating an empty array to store the data

        if self.modality == 'Radar':
            coordinates_columns = []

        self.data_matrix = np.zeros( list(matrix_dimensions.values()) )


        for _, row in self.dataframe.iterrows():

            self.data_matrix[ tuple( row[matrix_dimensions.keys()].astype(int).to_list() ) ] = row[matrix_value] if isinstance(matrix_value,str) else matrix_value

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


def cartesian_to_spherical(xyz: np.ndarray) -> pd.DataFrame:

    def calculate_elevation_angle():

        xy = np.sqrt( xyz[:,0]**2 + xyz[:,1]**2 )

        z = xyz[:,2]

        elevation_angle_defined = {'from Z-axis down': np.arctan2(xy, z),
                                    'from XY-plane up': np.arctan2(z, xy)}

        return elevation_angle_defined['from XY-plane up'] * 180 / 3.14


    assert xyz.shape[1] == 3, 'xyz must be a numpy array of shape (n,3)'


    from collections import namedtuple

    Desc = namedtuple('Desc', ['Range', 'Elevation', 'Azimuth'])

    ptsnew = np.hstack((xyz, np.zeros(xyz.shape)))

    # Range: sqrt(x^2 + y^2 + z^2)
    ptsnew[:,3] = np.sqrt( xyz[:,0]**2 + xyz[:,1]**2 + xyz[:,2]**2 )

    Desc.Range = ptsnew[:,3]

    # Elevation Angle: arctan( sqrt(x^2 + y^2) / z )
    ptsnew[:,4] = calculate_elevation_angle()

    Desc.Elevation = ptsnew[:,4]

    # Azimuth Angle: arctan(y/x)
    ptsnew[:,5] = np.arctan2(xyz[:,1], xyz[:,0]) * 180 / 3.14

    Desc.Azimuth = ptsnew[:,5]

    # Creating the dataframe
    ptsnew = pd.DataFrame(ptsnew, columns=['X', 'Y', 'Z', 'Range', 'Elevation Angle', 'Azimuth Angle'])

    return ptsnew, Desc


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
