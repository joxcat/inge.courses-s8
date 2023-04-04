package com.example.pokesocks13.fragments

import android.annotation.SuppressLint
import android.content.Context
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.core.content.res.ResourcesCompat
import androidx.databinding.ObservableArrayList
import androidx.fragment.app.Fragment
import com.example.pokesocks13.BuildConfig
import com.example.pokesocks13.Marker
import com.example.pokesocks13.R
import com.example.pokesocks13.models.Position
import com.example.pokesocks13.models.WildPokemon
import com.example.pokesocks13.models.WildPokemonListener
import org.osmdroid.config.Configuration
import org.osmdroid.tileprovider.tilesource.TileSourceFactory
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.Marker

class MapFragment : LocationListener, Fragment() {
    private lateinit var map: MapView
    private lateinit var userPosition: Position
    private lateinit var locationManager: LocationManager
    private lateinit var userMarker: Marker
    private lateinit var context: Context

    private lateinit var wildPokemons: ObservableArrayList<WildPokemon>
    private lateinit var wildPokemonsCache: ArrayList<WildPokemon>
    private lateinit var wildPokemonsListener: WildPokemonListener

    fun setWildPokemons(value: ObservableArrayList<WildPokemon>) {
        wildPokemons = value
        wildPokemonsCache = ArrayList(value)
    }

    fun setUserPosition(value: Position) {
        userPosition = value
    }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        context = inflater.context

        val config = Configuration.getInstance()
        config.userAgentValue = BuildConfig.APPLICATION_ID
        map = MapView(inflater.context)
        map.setTileSource(TileSourceFactory.MAPNIK)
        map.setMultiTouchControls(true)
        map.isClickable = true
        map.controller.setZoom(10.0)

        userMarker = Marker(map)
        wildPokemonsListener = WildPokemonListener(map, inflater.context, wildPokemonsCache)

        regenerateMarkers()
        focusLoc(userPosition)

        return map
    }

    override fun onResume() {
        super.onResume()
        map.onResume()

        registerLocationListener()
        registerWildPokemonsListener()
        regenerateMarkers()

        focusLoc(userPosition)
    }

    override fun onPause() {
        super.onPause()
        map.onPause()

        unregisterLocationListener()
        unregisterWildPokemonsListener()
    }

    private fun moveTo(pos: Position) {
        map.controller.animateTo(pos.toGeoPoint())
    }

    private fun focusLoc(pos: Position) {
        map.controller.let {
            it.setCenter(pos.toGeoPoint())
            it.setZoom(18.0)
        }
    }

    private fun regenerateMarkers() {
        map.overlays.clear()

        userMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_CENTER)
        userMarker.icon =
            ResourcesCompat.getDrawable(context.resources, R.drawable.pokeball, context.theme)
        userMarker.title = "Moi"
        userMarker.position = userPosition.toGeoPoint()

        map.overlays.add(userMarker)

        for (pokemon in wildPokemons) {
            map.overlays.add(Marker(context, map, pokemon))
        }
    }

    // -- Location handling --
    override fun onLocationChanged(location: Location) {
        val loc = Position(location)

        userPosition.update(loc)
        userMarker.position = loc.toGeoPoint()

        moveTo(loc)
    }

    // NOTE: We checked theses permissions before starting the fragment
    @SuppressLint("MissingPermission")
    private fun registerLocationListener() {
        locationManager =
            requireContext().getSystemService(Context.LOCATION_SERVICE) as LocationManager
        locationManager.requestLocationUpdates(
            LocationManager.GPS_PROVIDER,
            120.0.toLong(),
            10.0.toFloat(),
            this
        )
        locationManager.requestLocationUpdates(
            LocationManager.NETWORK_PROVIDER,
            1000.0.toLong(),
            100.0.toFloat(),
            this
        )
    }

    private fun unregisterLocationListener() {
        locationManager =
            requireContext().getSystemService(Context.LOCATION_SERVICE) as LocationManager
        locationManager.removeUpdates(this)
    }

    // -- Wild pokemons handling --
    private fun registerWildPokemonsListener() {
        wildPokemons.addOnListChangedCallback(wildPokemonsListener)
    }

    private fun unregisterWildPokemonsListener() {
        wildPokemons.removeOnListChangedCallback(wildPokemonsListener)
    }
}