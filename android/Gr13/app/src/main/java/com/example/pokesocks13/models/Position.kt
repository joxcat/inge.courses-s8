package com.example.pokesocks13.models

import android.location.Location
import androidx.databinding.BaseObservable
import org.osmdroid.util.GeoPoint
import kotlin.math.*

val ONE_DEG_MAGIC_NUMBER_IN_METERS = 10.0.pow(7) / 90

fun Position(position: Position): Position {
    return Position(position.latitude, position.longitude)
}

fun Position(loc: Location): Position {
    return Position(loc.latitude, loc.longitude)
}

fun Position(): Position {
    return Position(0.0, 0.0)
}

class Position(
    var latitude: Double = 0.0,
    var longitude: Double = 0.0,
) : BaseObservable() {
    fun offsetInMeters(x: Int, y: Int): Position = offsetInMeters(x.toDouble(), y.toDouble())
    fun offsetInMeters(x: Double, y: Double): Position {
        latitude = latitude.plus(y / ONE_DEG_MAGIC_NUMBER_IN_METERS)
        longitude = longitude.plus(x / ONE_DEG_MAGIC_NUMBER_IN_METERS / cos(inRadians(latitude)))

        notifyChange()
        return this
    }

    fun toGeoPoint(): GeoPoint {
        return GeoPoint(latitude, longitude)
    }

    fun update(pos: Position): Position {
        latitude = pos.latitude
        longitude = pos.longitude

        notifyChange()
        return this
    }

    private fun inRadians(nb: Double): Double {
        return nb * PI / 180.0
    }

    fun distanceInMeters(pos: Position): Double {
        // Source: https://geolake.com/code/distance
        // Source: https://en.wikipedia.org/wiki/Earth_radius
        return acos(
            sin(inRadians(pos.latitude)) * sin(inRadians(latitude))
                    + cos(inRadians(pos.latitude)) * cos(inRadians(latitude))
                    * cos(inRadians(longitude - pos.longitude))
        ) * 6371.0 * 1000
    }
}