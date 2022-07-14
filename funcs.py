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
from glob import glob
import tensorflow as tf
from tqdm import tqdm


class Optimization():

    def __init__(self, input_shape=None, mode='vector'):

        self.mode = mode
        self.model = self.architecture(input_shape=input_shape)


    def fit(self, epochs=5, batch_size=32, train=None, valid=None):

        # if self.mode == 'vector' or isinstance(train, np.ndarray) or isinstance(train, pd.DataFrame):
        self.model.fit(x=train, y=train, epochs=epochs, batch_size=batch_size, validation_data=(valid, valid))

        # else:
        #     self.model.fit(train, epochs=epochs, batch_size=batch_size, validation_data=valid)


    def architecture(self, input_shape: tuple=None):

        def vector_autoencoder():
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

        def matrix_autoencoder():
            input = layers.Input(shape=input_shape)

            # Encoder
            x = layers.BatchNormalization()(input)
            x = layers.Conv2D(16, (3, 3), activation='relu', padding='same')(x)
            x = layers.MaxPooling2D((2, 2), padding='same')(x)
            x = layers.BatchNormalization()(x)
            x = layers.Conv2D(8, (3, 3), activation='relu', padding='same')(x)
            x = layers.MaxPooling2D((2, 2), padding='same')(x)
            x = layers.BatchNormalization()(x)
            x = layers.Conv2D(8, (3, 3), activation='relu', padding='same')(x)
            encoded = layers.MaxPooling2D((2, 2), padding='same')(x)

            # at this point the representation is (4, 4, 8) i.e. 128-dimensional

            x = layers.Conv2D(8, (3, 3), activation='relu', padding='same')(encoded)
            x = layers.UpSampling2D((2, 2))(x)
            x = layers.BatchNormalization()(x)
            x = layers.Conv2D(8, (3, 3), activation='relu', padding='same')(x)
            x = layers.UpSampling2D((2, 2))(x)
            x = layers.BatchNormalization()(x)
            x = layers.Conv2D(16, (3, 3), activation='relu')(x)
            x = layers.UpSampling2D((2, 2))(x)
            decoded = layers.Conv2D(input_shape[-1], (3, 3), activation='sigmoid', padding='same')(x)

            # Autoencoder
            model = models.Model(input, decoded)
            model.compile(optimizer='adam', loss='binary_crossentropy')

            return model

        if self.mode == 'vector':
            return vector_autoencoder()

        elif self.mode == 'matrix':
            return matrix_autoencoder()


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


    def __init__(self, modality = 'Radar', data_type='vector', dataset_directory='/home/artin/Documents/10dB_CR_10deg_in_2021-10-15-14-15-49'):

        self.dataset_directory = dataset_directory
        self.modality = modality
        self.data_type = data_type


    def _read_dataframe(self, dir):
        self.dataframe_original = pd.read_csv(dir)
        self.dataframe_original.columns = self.ORIGINAL_COLUMNS[self.modality]
        self.dataframe = self.dataframe_original.copy()


    def get_dataframe(self, dir: str):

        self._read_dataframe(dir=dir)

        # Converting Lidar cartesian coordinates to spherical
        if self.modality == 'Lidar':
            self.dataframe = self.converting_lidar_to_looklike_radar(dataframe=self.dataframe)

        # Removing unnecessary columns
        self.dataframe = self.dataframe[ self.COLUMNS_RENAMING_MAPS.keys() ]

        # Rename columns
        self.dataframe.rename(columns=self.COLUMNS_RENAMING_MAPS, inplace=True)

        # remove_duplicates
        self.dataframe = self.remove_duplicated_bird_pov_points(df=self.dataframe.copy())

        # Normalize the data to > 0 according to SCALE_BIAS
        for name in set(self.SCALE_BIAS.keys())  &  set(self.dataframe.columns):
            self.dataframe[name] = self.dataframe[name] + self.SCALE_BIAS[name].Min


        # Removing the Lidar values that are outside pre-specified Radar range
        # if self.modality == 'Lidar':
        self._filtering_values_outside_specified_range()


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

        # Converting the dataframe to a matrix
        if self.data_type == 'matrix':
            return self.vector_to_matrix_convertion()

        return self.dataframe


    def vector_to_matrix_convertion(self):

        # Creating an empty array to store the data
        self.data_matrix = np.zeros( (self.SCALE_BIAS['Elevation'].Scale , self.SCALE_BIAS['Azimuth'].Scale , 3) )

        for _, row in self.dataframe.iterrows():

            # ToDo: Add 'Velocity' to the list of columns in Radar. This requires to make Radar and Lidar different dataframes
            self.data_matrix[ int(row.Elevation) , int(row.Azimuth), : ] = [ row['RCS'] , row['Range'] , 1 / (1 + row['Range']) ] # 'Velocity'

        return self.data_matrix


    @staticmethod
    def visualize(points=None, spherical=True, method='open3d'):

        # Converting the spherical coordinates to cartesian
        if spherical:
            df_xyz = DataLoader().spherical_to_cartesian(Azimuth_Angle=points.Azimuth , Elevation_Angle=points.Elevation , Range=points.Range)
            points = df_xyz.values
        else:
            points = points.values


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

    @staticmethod
    def converting_lidar_to_looklike_radar(dataframe: pd.DataFrame):

        ''' dataframe should contain ['X', 'Y', 'Z', 'RCS'] columns. '''

        for name in ['X','Y','Z','RCS']:
            assert name in dataframe.columns, f'{name} is not in the dataframe'


        # converting x,y,z to spherical angles/ranges
        Range, Elevation, Azimuth = DataLoader().cartesian_to_spherical(x=dataframe.X , y=dataframe.Y , z=dataframe.Z )

        # merging the spherical coordinates and the original dataframe
        df = dataframe.copy()

        # For details of the reason behind below calculations refer to the class docstring
        df['Azimuth Index']   = Azimuth.round()
        df['Elevation Index'] = Elevation.round().astype(int)
        df['Range Index']     = (Range / 0.5859).round().astype(int)
        df['Amplitude Index'] = np.power( df['RCS'] / 20.0 , 10 )

        return df

    @staticmethod
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

    @staticmethod
    def spherical_to_cartesian(Azimuth_Angle, Elevation_Angle, Range):

        Azimuth_Angle = Azimuth_Angle * 3.14 / 180
        Elevation_Angle = Elevation_Angle * 3.14 / 180

        x = Range * np.cos(Azimuth_Angle) * np.cos(Elevation_Angle)
        y = Range * np.sin(Azimuth_Angle) * np.cos(Elevation_Angle)
        z = Range * np.sin(Elevation_Angle)

        return pd.DataFrame({'X': x, 'Y': y, 'Z': z})

    @staticmethod
    def remove_duplicated_bird_pov_points(df: pd.DataFrame):

        # First we sort the values based on RCS
        df.sort_values(by=['RCS'], inplace=True)

        # Then we remove all the duplicate rows (similar Azimuth and Elevation) except for the
        # last occurance (which has the highest RCS due to previous step sorting)
        df.drop_duplicates( subset=['Azimuth','Elevation'], keep='last', inplace=True)

        # Finaly sort the dataframe back to its original index order
        df.sort_index(inplace=True)

        return df


class VectorInput(DataLoader, Optimization):

    def __init__(self, dataset_directory='/home/artin/Documents/10dB_CR_10deg_in_2021-10-15-14-15-49' , modality='Radar', normalize=True, filename='1_.txt'):

        # Loading the data
        DataLoader.__init__( self, data_type='vector' , dataset_directory=dataset_directory, modality=modality)

        self.get_data(filename=filename)

        self.data = self.split_dataset(dataframe=self.dataframe.copy())

        # Getting the architecture of the model
        Optimization.__init__( self, input_shape=(self.data.train.shape[1],1) )

        # Training the model
        self.fit(epochs=5, batch_size=32, train=self.data.train, valid=self.data.valid)

        # Testing the model
        self.prediction = self.predict(test=self.data.test)


    @staticmethod
    def split_dataset(dataframe, train_valid_test_ratio=[3,1,1]):

        if train_valid_test_ratio is None:
            return

        frac = {}
        for ix, mode in enumerate(['train' , 'valid' , 'test']):
            frac[mode] = train_valid_test_ratio[ix]/sum(train_valid_test_ratio)


        train = dataframe.sample(frac=frac['train'], random_state=42)
        dataframe.drop(train.index, inplace=True)

        valid = dataframe.sample(frac=frac['valid'], random_state=42)
        dataframe.drop(valid.index, inplace=True)

        data = namedtuple('data', ['train', 'valid', 'test'])
        return data(train=train, valid=valid, test=dataframe)


class MatrixInput(DataLoader, Optimization):

    def __init__(self, dataset_directory='/home/artin/Documents/10dB_CR_10deg_in_2021-10-15-14-15-49' , modality='Radar', normalize=False):

        # Loading the data
        DataLoader.__init__(self, data_type='matrix' , modality=modality, dataset_directory=dataset_directory)

        tfdataset, dataset = self.load_dataset(max_limit=2000)

        self.data = self.split_dataset(tfdataset=tfdataset)

        # dataset /= 100
        self.dataset = dataset

        # Getting the architecture of the model
        input_shape = list(self.data.train.element_spec[0].shape)
        Optimization.__init__(self, input_shape=input_shape, mode='matrix')

        # Training the model
        # self.fit(epochs=5, batch_size=32, train=self.data.train, valid=self.data.valid)
        self.fit(epochs=30, batch_size=16, train=dataset, valid=dataset)

        # Testing the model
        # self.predictions = self.predict(test=self.data.test)
        self.predictions = self.predict(test=dataset)


    def load_dataset(self, max_limit=None):

        def _add_channel_dimention(dataset):
            if len(dataset.shape) == 3:
                dataset = dataset[... , np.newaxis]
            return dataset

        def _normalize_each_channel(dataset):
            for ch in range(dataset.shape[3]):
                dataset[...,ch] = dataset[...,ch] - dataset[...,ch].min()
                dataset[...,ch] = dataset[...,ch] / dataset[...,ch].max()
            return dataset

        def _create_tensorflow_dataset(dataset, list_frames):

            # tensorflow dataset from dataframe
            tfdataset_data  = tf.data.Dataset.from_tensor_slices(dataset)
            tfdataset_label = tf.data.Dataset.from_tensor_slices(dataset)
            tfdataset = tf.data.Dataset.zip((tfdataset_data, tfdataset_label))
            tfdataset.shuffle(seed=10, buffer_size=len(list_frames))
            # tfdataset.batch(batch_size=32)

            # view the values in dataset
            # for x in tfdataset.take(3):
            #     print(x.shape)

            return tfdataset

        def _get_list_of_samples():
            list_frames = glob(f'{self.dataset_directory}/{self.modality}/*.txt')

            list_frames = [i.split(sep='/')[-1] for i in list_frames]

            if max_limit is not None:
                list_frames = list_frames[:max_limit]

            return list_frames

        def _get_dataset(list_frames):

            for i, filename in tqdm(enumerate(list_frames), desc='Loading the dataset'):

                if i ==0:
                    dt = self.get_data(filename=filename)
                    dataset = np.zeros( (len(list_frames),) + dt.shape )
                    dataset[i,:,:,:] = dt
                else:

                    dataset[i,:,:,:] = self.get_data(filename=filename)

            return dataset


        list_frames = _get_list_of_samples()

        dataset   = _get_dataset(list_frames)

        dataset   = _add_channel_dimention(dataset)

        dataset   = _normalize_each_channel(dataset)

        tfdataset = _create_tensorflow_dataset(dataset, list_frames)

        return tfdataset, dataset

    @staticmethod
    def split_dataset(tfdataset, train_valid_test_ratio=[3,1,1]):

        if train_valid_test_ratio is None:
            return

        frac = {}
        for ix, mode in enumerate(['train' , 'valid' , 'test']):
            ratio = train_valid_test_ratio[ix]/sum(train_valid_test_ratio)
            frac[mode] = int(ratio * tfdataset.cardinality().numpy())


        train = tfdataset.take(frac['train'])
        valid = tfdataset.skip(frac['train']).take(frac['valid'])
        test  = tfdataset.skip(frac['train']).skip(frac['valid'])

        data = namedtuple('data', ['train', 'valid', 'test'])
        return data(train=train, valid=valid, test=test)

    @staticmethod
    def extract_spherical_coordinates(sample: np.ndarray):
        ''' sample input should be a 3D array with axes Elevation * Azimuth * Features
        which Features is currently set to be [RCS, Ranage, 100/Range'''

        # Find the non-zero coordinates in Range channel
        non_zero_coordinates = np.nonzero(sample[:,:,0])

        # get the values at the non zero coordinates
        RCS      = sample[:,:,0][non_zero_coordinates]
        Range    = sample[:,:,1][non_zero_coordinates]
        invRange = sample[:,:,2][non_zero_coordinates]

        return pd.DataFrame( non_zero_coordinates + (RCS, Range, invRange)  , index=['Elevation','Azimuth','RCS','Range', 'invRange']).T




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
