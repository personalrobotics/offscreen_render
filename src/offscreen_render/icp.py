#!/usr/bin/env python
import numpy as np

def iterative_closest_point(input_model_points, sensor_points, initial_transform, iters=100, match_threshold=0.2, exit_threshold=0.01):
    model_points = np.array([input_model_points], copy=True);
    model_points.reshape()
    num_model_points = model_points.shape[1];
    transform = initial_transform;
    error_threshold = exit_threshold * num_model_points;
    sum_squared_error = error_threshold + 1;

    while i < iters or sum_squared_error < exit_threshold:
        matches, sum_squared_error = compute_matches(model_points, sensor_points, transform, match_threshold);

        if(!len(matches)) :
            break;

        new_transform = least_square_transform(model_points, sensor_points, matches);
        model_points = transform_cloud(model_points, transform)
        transform = np.dot(new_transform, transform);
        i++;
    return transform, sum_squared_error;

def transform_cloud(cloud, transform):
    R = transform[0:3, 0:3];
    t = transform[0:3, 3];
    return np.dot(R, cloud) + t;

def compute_matches(model_points, sensor_points, transform, match_threshold):
    return compute_matches_brute_force(model_points, sensor_points, transform, match_threshold);

def compute_matches_brute_force(model_points, sensor_points, transform, match_threshold):
    num_model_points = model_points.shape[1];
    num_sensor_points = sensor_points.shape[1];
    matches = [];
    errors = np.array([0] * num_model_points);

    for i in xrange(0, num_model_points):
        closestDist = match_threshold + 1;
        closest = -1;
        for j in xrange(0, num_sensor_points):
            diff = model_points[i, :] - sensor_points[i, :];
            dist = np.dot(np.transpose(diff), diff);

            if(dist < closestDist and dist < match_threshold):
                closest = j;
                errors[i] = dist;
        if (closest > 0):
            matches.append((i, j));
    return closest, np.sum(errors);


def least_square_transform(model_points, sensor_points, matches):
    sum_model = np.array([0, 0, 0]);
    sum_data = np.array([0, 0, 0]);
    sum_mat = np.zeros((3, 3));

    k = 0;
    for (i, j) in matches:
        sum_model += model_matches[:, k];
        sum_data += sensor_matches[:, k];
        sum_mat += np.dot(sensor_matches[:, k], np.transpose(model_matches[:, k]))
        k++;

    avg_model_point = sum_model / len(matches);
    avg_data_point = sum_data / len(matches);
    sum_mat -= (1.0 / len(matches)) * np.dot(sum_data, np.transpose(sum_model));

    U, S, V = np.linalg.svd(sum_mat);
    rotation = np.dot(V, np.tranpose(U));

    avg_shift = avg_data_point - np.tranpose(rotation) * avg_model_point;
    rot_inv = np.tranpose(rotation);

    to_return = np.identity(4);
    to_return[0:3, 0:3] = rot_inv;
    to_return[0:3, 3] = avg_shift;
    return to_return;